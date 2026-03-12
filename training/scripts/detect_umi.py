#!/usr/bin/env python3
"""
detect_umi.py — Stabilized Offline UMI Pose Detection
Pipeline: data/chunked/ → detect_umi.py → data/processed/
"""

import argparse, glob, os, sys
import cv2, cv2.aruco as aruco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_scipy, Slerp
from tqdm import tqdm
import bisect

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.reader import read_ros2_messages
from rosbags.typesys import Stores, get_typestore

# ── ROS type system ───────────────────────────────────────────────────────────
typestore   = get_typestore(Stores.ROS2_HUMBLE)
Time        = typestore.types['builtin_interfaces/msg/Time']
Header      = typestore.types['std_msgs/msg/Header']
Quat        = typestore.types['geometry_msgs/msg/Quaternion']
Vector3      = typestore.types['geometry_msgs/msg/Vector3']
Transform    = typestore.types['geometry_msgs/msg/Transform']
TransformStamped = typestore.types['geometry_msgs/msg/TransformStamped']
TFMessage    = typestore.types['tf2_msgs/msg/TFMessage']

# ── Config ────────────────────────────────────────────────────────────────────
MARKER_YAML    = os.path.expanduser("~/VTAM/src/vtam_core/config/teleop_april_marker_info_86mm.yaml")
SYNC_TOPIC     = "/sync_pulse"
IMAGE_TOPIC    = "/camera/color/image_raw/compressed"
CAM_INFO_TOPIC = "/camera/color/camera_info"
TF_TOPIC       = "/tf"
GRIPPER_OFFSET = np.array([0.242, 0.0, 0.0])

# ── Helpers ───────────────────────────────────────────────────────────────────

def smooth_trajectory_data(det_map, window=7):
    """Applies a weighted moving average to positions and quaternions to kill handover jitter."""
    ts_list = sorted(det_map.keys())
    if len(ts_list) < window: return det_map
    
    smoothed_map = {}
    for i, t in enumerate(ts_list):
        start, end = max(0, i - window // 2), min(len(ts_list), i + window // 2 + 1)
        p_window = [det_map[ts_list[j]][0] for j in range(start, end)]
        q_window = [det_map[ts_list[j]][1] for j in range(start, end)]
        
        avg_p = np.mean(p_window, axis=0)
        ref_q = q_window[0]
        aligned_q = [q if np.dot(ref_q, q) >= 0 else -q for q in q_window]
        avg_q = np.mean(aligned_q, axis=0)
        avg_q /= np.linalg.norm(avg_q)
        
        # Recalculate gripper based on smoothed pose
        avg_grip_p = avg_p + R_scipy.from_quat(avg_q).apply(GRIPPER_OFFSET)
        smoothed_map[t] = (avg_p, avg_q, avg_grip_p)
    return smoothed_map

def make_tf_message(t_ns, parent, child, pos, quat):
    sec, nsec = int(t_ns // 10**9), int(t_ns % 10**9)
    msg = TFMessage(transforms=[TransformStamped(
        header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=parent),
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            rotation=Quat(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
        ))])
    return typestore.serialize_cdr(msg, 'tf2_msgs/msg/TFMessage')

# ── Math & Filters ────────────────────────────────────────────────────────────

class OneEuroFilter:
    def __init__(self, freq, min_cutoff=0.01, beta=0.1): # Tuned for offline stability
        self.freq, self.min_cutoff, self.beta = freq, min_cutoff, beta
        self.x_prev = self.dx_prev = None

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev, self.dx_prev = x, np.zeros_like(x)
            return x
        te, dx = 1.0 / self.freq, (x - self.x_prev) * self.freq
        a_d = te / (te + 1.0 / (2 * np.pi * 1.0))
        edx = self.dx_prev + a_d * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        a = te / (te + 1.0 / (2 * np.pi * cutoff))
        self.x_prev = self.x_prev + a * (x - self.x_prev)
        self.dx_prev = edx
        return self.x_prev

class AprilTagMarker:
    def __init__(self, tag_id, marker_info):
        self.info = marker_info.get(str(tag_id), {'length_mm': 64.0, 'frames': {}})
        self.length_mm = self.info.get('length_mm', 64.0)
        self.pos, self.axes = None, None

    def update(self, corners, K, D):
        half = self.length_mm / 2.0
        pts = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        _, rvec, tvec = cv2.solvePnP(pts, corners, K, D)
        self.pos = tvec.flatten() / 1000.0
        R = cv2.Rodrigues(rvec)[0]
        self.axes = [-R[:3, 1], -R[:3, 0], -R[:3, 2]]

def cube_pose_from_tag(marker, config):
    t = config.get('trans', [0, 0, 0])
    pos = marker.pos + t[0]*marker.axes[0] + t[1]*marker.axes[1] + t[2]*marker.axes[2]
    R_tag, R_offset = np.column_stack(marker.axes), R_scipy.from_quat(config.get('quat', [0, 0, 0, 1])).as_matrix()
    return pos, R_scipy.from_matrix(R_tag @ R_offset.T).as_quat()

def avg_quaternions(quats, weights):
    if not quats: return np.array([0, 0, 0, 1])
    quats = [q / np.linalg.norm(q) for q in quats]
    aligned = [quats[0]]
    for i in range(1, len(quats)):
        aligned.append(quats[i] if np.dot(quats[0], quats[i]) >= 0 else -quats[i])
    Q = sum(w * np.outer(q, q) for q, w in zip(aligned, weights))
    return np.linalg.eigh(Q)[1][:, -1]

# ── Static Kinematics ────────────────────────────────────────────────────────

def static_base_to_head():
    t1 = np.array([-0.067878, 0.133880, 0.026018])
    r1 = R_scipy.from_euler('xyz', [1.579757, -0.004523, -0.002246])
    t2, r2 = np.array([0.0, 1.33, 0.0]), R_scipy.from_euler('xyz', [1.57079, -1.57079, 3.1416])
    return t1 + r1.apply(t2), r1 * r2

def static_head_tilt_to_cam():
    t1, r1 = np.array([0.0259, -0.0095, 0.0174]), R_scipy.from_euler('xyz', [0.0045, -0.0028, 0.0168])
    t2, r2 = np.array([0.0106, 0.0175, 0.0125]), R_scipy.identity()
    t3, r3 = np.array([0.0, 0.015, 0.0]), R_scipy.identity()
    r4 = R_scipy.from_euler('xyz', [-np.pi/2, 0, -np.pi/2])
    r123 = r1 * r2 * r3
    return t1 + r1.apply(t2) + (r1*r2).apply(t3), r123 * r4

# ── Core Processing ───────────────────────────────────────────────────────────

def process_episode(in_path, out_path, marker_info, args):
    K, D, sync_ns, imgs = None, None, [], []
    tf_pan_raw, tf_pan_ts, tf_tilt_raw, tf_tilt_ts = [], [], [], []

    with open(in_path, 'rb') as f:
        r = make_reader(f)
        for _, channel, message in r.iter_messages(topics=[SYNC_TOPIC]):
            sync_ns.append(message.publish_time)

    for msg in read_ros2_messages(in_path, topics=[CAM_INFO_TOPIC, IMAGE_TOPIC, TF_TOPIC]):
        t, topic = msg.publish_time_ns, msg.channel.topic
        if topic == CAM_INFO_TOPIC and K is None:
            K, D = np.array(msg.ros_msg.k).reshape(3, 3), np.array(msg.ros_msg.d)
        elif topic == IMAGE_TOPIC:
            imgs.append((msg.ros_msg.data, t, t / 1e9))
        elif topic == TF_TOPIC:
            for tf in msg.ros_msg.transforms:
                val = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z,
                       tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]
                if tf.child_frame_id == 'link_head_pan':
                    tf_pan_raw.append(val); tf_pan_ts.append(t / 1e9)
                elif tf.child_frame_id == 'link_head_tilt':
                    tf_tilt_raw.append(val); tf_tilt_ts.append(t / 1e9)
    if K is None or len(sync_ns) < 10: return False
    
    pan_data, pan_ts = np.array(tf_pan_raw), np.array(tf_pan_ts)
    tilt_data, tilt_ts = np.array(tf_tilt_raw), np.array(tf_tilt_ts)
    t_bh, R_bh = static_base_to_head()
    t_tc, R_tc = static_head_tilt_to_cam()

    detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11), aruco.DetectorParameters())
    detector.getDetectorParameters().cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    
    pos_filt, quat_filt = OneEuroFilter(30.0, args.pos_min_cutoff, args.pos_beta), OneEuroFilter(30.0, args.quat_min_cutoff, args.quat_beta)
    collection, prev_quat, det_map = {}, None, {}

    for compressed, t_ns, t_sec in tqdm(imgs, desc=" Detecting", leave=False):
        img = cv2.imdecode(np.frombuffer(compressed, np.uint8), cv2.IMREAD_COLOR)
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None: continue

        candidates = []
        for c, aid in zip(corners, ids.flatten()):
            aid = int(aid)
            if aid not in collection: collection[aid] = AprilTagMarker(aid, marker_info)
            m = collection[aid]; m.update(c[0], K, D)
            if 'umi_cube' in m.info.get('frames', {}):
                pos, quat = cube_pose_from_tag(m, m.info['frames']['umi_cube'])
                candidates.append((pos, quat, np.clip(cv2.contourArea(c[0].reshape(4,2))/1000.0, 0.1, 1.0)))

        if not candidates: continue
        w = np.array([c[2] for c in candidates]); w /= w.sum()
        pos_cam, quat_cam = np.average([c[0] for c in candidates], axis=0, weights=w), avg_quaternions([c[1] for c in candidates], w)
        
        # Kinematics & Pitch Comp
        p_tf = pan_data[np.clip(np.searchsorted(pan_ts, t_sec)-1, 0, len(pan_data)-1)]
        t_tf = tilt_data[np.clip(np.searchsorted(tilt_ts, t_sec)-1, 0, len(tilt_data)-1)]
        
        R_bt = R_bh * R_scipy.from_quat(p_tf[3:7]) * R_scipy.from_quat(t_tf[3:7])
        t_bt = t_bh + R_bh.apply(p_tf[:3]) + (R_bh * R_scipy.from_quat(p_tf[3:7])).apply(t_tf[:3])
        R_bc, t_bc = R_bt * R_tc, t_bt + R_bt.apply(t_tc)

        pos_base = t_bc + R_bc.apply(pos_cam)
        R_fix = R_scipy.from_euler('zyx', [0, -R_bc.as_euler('xyz', degrees=True)[1], 180], degrees=True)
        quat_base = (R_bc * R_scipy.from_quat(quat_cam) * R_fix).as_quat()

        if prev_quat is not None and np.dot(quat_base, prev_quat) < 0: quat_base = -quat_base
        pos_base, quat_base = pos_filt.filter(pos_base), quat_filt.filter(quat_base)
        quat_base /= np.linalg.norm(quat_base)
        prev_quat = quat_base.copy()

        quat_final = (R_scipy.from_quat(quat_base) * R_scipy.from_euler('z', 90, degrees=True)).as_quat()
        det_map[t_ns] = (pos_base, quat_final, pos_base + R_scipy.from_quat(quat_final).apply(GRIPPER_OFFSET))

    # if len(det_map) > 10:
    #     det_map = smooth_trajectory_data(det_map, window=args.smooth_window)

    # Write Output
    with open(in_path, "rb") as f_in, open(out_path, "wb") as f_out:
        reader, writer = make_reader(f_in), Writer(f_out)
        writer.start()
        summary = reader.get_summary()
        c_map = {cid: writer.register_channel(topic=ch.topic, message_encoding=ch.message_encoding, schema_id=writer.register_schema(name=summary.schemas[ch.schema_id].name, encoding=summary.schemas[ch.schema_id].encoding, data=summary.schemas[ch.schema_id].data)) for cid, ch in summary.channels.items()}
        tf_sid = writer.register_schema(name="tf2_msgs/msg/TFMessage", encoding="ros2msg", data=typestore.generate_msgdef('tf2_msgs/msg/TFMessage')[0].encode())
        tf_ch = writer.register_channel(topic="/tf", message_encoding="cdr", schema_id=tf_sid)

        for _, channel, msg in reader.iter_messages():
            writer.add_message(c_map[channel.id], msg.log_time, msg.data, msg.publish_time)
            if channel.topic == IMAGE_TOPIC and msg.publish_time in det_map:
                p, q, _ = det_map[msg.publish_time]
                writer.add_message(tf_ch, msg.log_time, make_tf_message(msg.publish_time, "base_link", "umi_disconnect", p, q), msg.publish_time)
                writer.add_message(tf_ch, msg.log_time, make_tf_message(msg.publish_time, "umi_disconnect", "umi_gripper", GRIPPER_OFFSET, [0,0,0,1]), msg.publish_time)
        writer.finish()
    return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('demo_name', type=str)
    parser.add_argument('--pos-min-cutoff', type=float, default=0.05)
    parser.add_argument('--pos-beta', type=float, default=0.5)
    parser.add_argument('--quat-min-cutoff', type=float, default=0.01)
    parser.add_argument('--quat-beta', type=float, default=0.1)
    parser.add_argument('--smooth-window', type=int, default=11)
    args = parser.parse_args()

    with open(MARKER_YAML, 'r') as f: marker_info = yaml.safe_load(f)
    paths = sorted(glob.glob(os.path.expanduser(f"~/VTAM/data/chunked/{args.demo_name}/*.mcap")))
    for p in paths:
        out = p.replace('chunked', 'processed')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        print(f"Processing {os.path.basename(p)}...")
        process_episode(p, out, marker_info, args)

if __name__ == '__main__': main()