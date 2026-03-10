#!/usr/bin/env python3
"""
detect_umi.py — Offline UMI pose detection.

    data/chunked/ → detect_umi.py → data/processed/

Reads chunked episode mcaps, runs AprilTag detection on every frame at CPU speed,
writes new mcaps with all original data + injected /umi_cube_pose, /umi_gripper_pose,
/umi_trajectory topics.

Usage:
    python3 detect_umi.py pickup_cup
    python3 detect_umi.py pickup_cup --episode 3 --force

Requires: pip install rosbags
"""

import argparse, glob, os, sys
import cv2, cv2.aruco as aruco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_scipy
from tqdm import tqdm

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.reader import read_ros2_messages
from rosbags.serde import serialize_cdr
from rosbags.typesys import Stores, get_typestore

# ── ROS type system (handles all serialization) ───────────────────────────────

typestore = get_typestore(Stores.ROS2_HUMBLE)
Time       = typestore.types['builtin_interfaces/msg/Time']
Header     = typestore.types['std_msgs/msg/Header']
Point      = typestore.types['geometry_msgs/msg/Point']
Quat       = typestore.types['geometry_msgs/msg/Quaternion']
Pose       = typestore.types['geometry_msgs/msg/Pose']
PoseStamped = typestore.types['geometry_msgs/msg/PoseStamped']
NavPath    = typestore.types['nav_msgs/msg/Path']

# ── Config ─────────────────────────────────────────────────────────────────────

MARKER_YAML    = os.path.expanduser("~/VTAM/install/vtam_core/share/vtam_core/config/teleop_april_marker_info_86mm.yaml")
SYNC_TOPIC     = "/sync_pulse"
IMAGE_TOPIC    = "/camera/color/image_raw/compressed"
CAM_INFO_TOPIC = "/camera/color/camera_info"
TF_TOPIC       = "/tf"
CAM_TF_PARENT  = "base_link"
CAM_TF_CHILD   = "camera_color_optical_frame"
GRIPPER_OFFSET = np.array([0.242, 0.0, 0.0])

# ── Message builders (rosbags handles CDR serialization) ──────────────────────

def make_pose_stamped(t_ns, frame_id, pos, quat):
    sec, nsec = int(t_ns // 10**9), int(t_ns % 10**9)
    msg = PoseStamped(
        header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id),
        pose=Pose(
            position=Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
            orientation=Quat(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
        ),
    )
    return serialize_cdr(msg, 'geometry_msgs/msg/PoseStamped')


def make_path(t_ns, frame_id, poses):
    sec, nsec = int(t_ns // 10**9), int(t_ns % 10**9)
    msg = NavPath(
        header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame_id),
        poses=[
            PoseStamped(
                header=Header(stamp=Time(sec=int(pt // 10**9), nanosec=int(pt % 10**9)), frame_id=''),
                pose=Pose(
                    position=Point(x=float(p[0]), y=float(p[1]), z=float(p[2])),
                    orientation=Quat(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])),
                ),
            )
            for pt, p, q in poses
        ],
    )
    return serialize_cdr(msg, 'nav_msgs/msg/Path')

# ── Detection math (unchanged from UmiDetectorNode) ───────────────────────────

class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq, self.min_cutoff, self.beta, self.d_cutoff = freq, min_cutoff, beta, d_cutoff
        self.x_prev = self.dx_prev = None

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev, self.dx_prev = x, np.zeros_like(x)
            return x
        te = 1.0 / self.freq
        dx = (x - self.x_prev) * self.freq
        a_d = te / (te + 1.0 / (2 * np.pi * self.d_cutoff))
        edx = self.dx_prev + a_d * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        a = te / (te + 1.0 / (2 * np.pi * cutoff))
        x_filtered = self.x_prev + a * (x - self.x_prev)
        self.x_prev, self.dx_prev = x_filtered, edx
        return x_filtered


class AprilTagMarker:
    def __init__(self, tag_id, marker_info):
        self.info = marker_info.get(str(tag_id), {'length_mm': 64.0, 'frames': {}})
        self.length_mm = self.info.get('length_mm', 64.0)
        self.pos, self.axes = None, [None, None, None]

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
    R_tag = np.column_stack(marker.axes)
    R_offset = R_scipy.from_quat(config.get('quat', [0, 0, 0, 1])).as_matrix()
    R_cube = R_tag @ R_offset.T
    return pos, R_scipy.from_matrix(R_cube).as_quat()


def avg_quaternions(quats, weights):
    if not quats: return np.array([0, 0, 0, 1])
    quats = [q / np.linalg.norm(q) for q in quats]
    aligned = [q if np.dot(quats[0], q) >= 0 else -q for q in quats]
    Q = sum(w * np.outer(q, q) for q, w in zip(aligned, weights))
    return np.linalg.eigh(Q)[1][:, -1]


def tag_weight(corners):
    return np.clip(cv2.contourArea(corners.reshape(4, 2)) / 1000.0, 0.1, 1.0)


# ── TF lookup ─────────────────────────────────────────────────────────────────

def build_tf_table(data, ts):
    order = np.argsort(ts)
    return np.array(data)[order], np.array(ts)[order]

def lookup_tf(data, ts, t):
    return data[np.clip(np.searchsorted(ts, t) - 1, 0, len(data) - 1)]


# ── Core: detect + write ──────────────────────────────────────────────────────

def process_episode(in_path, out_path, marker_info, args):
    """Read chunked mcap, detect on every frame, write processed mcap."""

    # ── Pass 1: Ingest all data ───────────────────────────────────────────────
    K, D = None, None
    sync_ns, imgs, tf_raw, tf_ts = [], [], [], []

    for msg in read_ros2_messages(in_path, topics=[SYNC_TOPIC, IMAGE_TOPIC, CAM_INFO_TOPIC, TF_TOPIC]):
        t = msg.publish_time_ns
        topic = msg.channel.topic

        if topic == SYNC_TOPIC:
            s = msg.ros_msg.header.stamp
            sync_ns.append(s.sec * 10**9 + s.nanosec)
        elif topic == CAM_INFO_TOPIC and K is None:
            K, D = np.array(msg.ros_msg.k).reshape(3, 3), np.array(msg.ros_msg.d)
        elif topic == IMAGE_TOPIC:
            imgs.append((msg.ros_msg.data, t, t / 1e9))
        elif topic == TF_TOPIC:
            for tf in msg.ros_msg.transforms:
                if tf.header.frame_id == CAM_TF_PARENT and tf.child_frame_id == CAM_TF_CHILD:
                    tr, ro = tf.transform.translation, tf.transform.rotation
                    tf_raw.append([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w])
                    tf_ts.append(t / 1e9)

    if K is None or len(sync_ns) < 30 or not tf_raw:
        print(f"    Insufficient data (cam={K is not None}, sync={len(sync_ns)}, tf={len(tf_raw)})")
        return False

    tf_data, tf_stamps = build_tf_table(tf_raw, tf_ts)

    # ── Pass 2: Detect on every frame ─────────────────────────────────────────
    detector = aruco.ArucoDetector(
        aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11),
        _detector_params(),
    )
    pos_filt = OneEuroFilter(30.0, args.pos_min_cutoff, args.pos_beta)
    quat_filt = OneEuroFilter(30.0, args.quat_min_cutoff, args.quat_beta)
    collection, prev_quat = {}, None
    # det_map: frame_timestamp_ns → (disconnect_pos, disconnect_quat, gripper_pos)
    det_map = {}

    for compressed, t_ns, t_sec in tqdm(imgs, desc="    Detecting", leave=False):
        img = cv2.imdecode(np.frombuffer(compressed, np.uint8), cv2.IMREAD_COLOR)
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue

        candidates = []
        for c, aid in zip(corners, ids.flatten()):
            aid = int(aid)
            if aid not in collection:
                collection[aid] = AprilTagMarker(aid, marker_info)
            m = collection[aid]
            m.update(c[0], K, D)
            frames = m.info.get('frames', {})
            if 'umi_cube' in frames:
                pos, quat = cube_pose_from_tag(m, frames['umi_cube'])
                candidates.append((pos, quat, tag_weight(c[0])))

        if not candidates:
            continue

        # Weighted fusion
        w = np.array([c[2] for c in candidates])
        w /= w.sum()
        pos_cam = np.average([c[0] for c in candidates], axis=0, weights=w)
        quat_cam = avg_quaternions([c[1] for c in candidates], w)

        # Camera → base_link
        tf = lookup_tf(tf_data, tf_stamps, t_sec)
        R_bc = R_scipy.from_quat(tf[3:7])
        pos_base = tf[:3] + R_bc.apply(pos_cam)

        pitch = R_bc.as_euler('xyz', degrees=True)[1]
        R_fix = R_scipy.from_euler('zyx', [0, -pitch, 180], degrees=True)
        quat_base = (R_bc * R_scipy.from_quat(quat_cam) * R_fix).as_quat()

        # Filter
        pos_base = pos_filt.filter(pos_base)
        if prev_quat is not None and np.dot(quat_base, prev_quat) < 0:
            quat_base = -quat_base
        quat_base = quat_filt.filter(quat_base)
        quat_base /= np.linalg.norm(quat_base)
        prev_quat = quat_base.copy()

        # Z 90° correction
        quat_corrected = (R_scipy.from_quat(quat_base) * R_scipy.from_euler('z', 90, degrees=True)).as_quat()

        # Gripper projection
        grip_pos = pos_base + R_scipy.from_quat(quat_corrected).apply(GRIPPER_OFFSET)

        det_map[t_ns] = (pos_base.copy(), quat_corrected.copy(), grip_pos.copy())

    n_det = len(det_map)
    pct = 100 * n_det / max(len(imgs), 1)
    print(f"    {n_det}/{len(imgs)} detections ({pct:.0f}%)")

    if n_det < 30:
        print(f"    Too few detections, skipping")
        return False

    # ── Pass 3: Copy original + inject detections ─────────────────────────────
    sync_set = set(sync_ns)
    trajectory = []  # accumulates (t_ns, pos, quat) for Path messages

    with open(in_path, "rb") as f_in, open(out_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)
        writer.start()

        # Mirror original channels
        s_map, c_map = {}, {}
        summary = reader.get_summary()
        for sid, s in summary.schemas.items():
            s_map[sid] = writer.register_schema(name=s.name, encoding=s.encoding, data=s.data)
        for cid, ch in summary.channels.items():
            c_map[cid] = writer.register_channel(
                topic=ch.topic, message_encoding=ch.message_encoding,
                schema_id=s_map[ch.schema_id], metadata=dict(ch.metadata),
            )

        # New channels (use ros2msg encoding with schema from rosbags)
        pose_schema = writer.register_schema(
            name="geometry_msgs/msg/PoseStamped", encoding="ros2msg",
            data=typestore.generate_msgdef('geometry_msgs/msg/PoseStamped')[0].encode(),
        )
        path_schema = writer.register_schema(
            name="nav_msgs/msg/Path", encoding="ros2msg",
            data=typestore.generate_msgdef('nav_msgs/msg/Path')[0].encode(),
        )
        cube_ch = writer.register_channel(topic="/umi_cube_pose", message_encoding="cdr", schema_id=pose_schema)
        grip_ch = writer.register_channel(topic="/umi_gripper_pose", message_encoding="cdr", schema_id=pose_schema)
        traj_ch = writer.register_channel(topic="/umi_trajectory", message_encoding="cdr", schema_id=path_schema)

        for _, channel, message in reader.iter_messages():
            ts = message.publish_time

            # Passthrough
            if channel.id in c_map:
                writer.add_message(channel_id=c_map[channel.id], log_time=message.log_time,
                                   data=message.data, publish_time=ts)

            # Inject at camera frames with detections
            if channel.topic == IMAGE_TOPIC and ts in det_map:
                d_pos, d_quat, g_pos = det_map[ts]
                writer.add_message(channel_id=cube_ch, log_time=ts,
                                   data=make_pose_stamped(ts, "base_link", d_pos, d_quat), publish_time=ts)
                writer.add_message(channel_id=grip_ch, log_time=ts,
                                   data=make_pose_stamped(ts, "base_link", g_pos, d_quat), publish_time=ts)
                trajectory.append((ts, d_pos, d_quat))

            # Trajectory at sync pulses
            if channel.topic == SYNC_TOPIC and ts in sync_set and trajectory:
                writer.add_message(channel_id=traj_ch, log_time=ts,
                                   data=make_path(ts, "base_link", trajectory), publish_time=ts)

        writer.finish()
    return True


def _detector_params():
    p = aruco.DetectorParameters()
    p.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 23
    p.adaptiveThreshWinSizeStep = 10
    return p


# ── CLI ────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Offline UMI detection: chunked → processed")
    parser.add_argument('demo_name', type=str)
    parser.add_argument('--episode', type=int, default=None)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--pos-min-cutoff', type=float, default=0.1)
    parser.add_argument('--pos-beta', type=float, default=3.0)
    parser.add_argument('--quat-min-cutoff', type=float, default=0.1)
    parser.add_argument('--quat-beta', type=float, default=3.0)
    args = parser.parse_args()

    with open(MARKER_YAML, 'r') as f:
        marker_info = yaml.safe_load(f)

    chunked_dir = os.path.expanduser(f"~/VTAM/data/chunked/{args.demo_name}")
    out_dir = os.path.expanduser(f"~/VTAM/data/processed/{args.demo_name}")
    os.makedirs(out_dir, exist_ok=True)

    if args.episode is not None:
        paths = [os.path.join(chunked_dir, f"episode_{args.episode:03d}.mcap")]
    else:
        paths = sorted(glob.glob(os.path.join(chunked_dir, "episode_*.mcap")))

    if not paths:
        print(f"No episodes in {chunked_dir}"); sys.exit(1)

    print(f"{len(paths)} episode(s): {chunked_dir} → {out_dir}\n")

    for p in paths:
        name = os.path.basename(p)
        out = os.path.join(out_dir, name)
        if os.path.exists(out) and not args.force:
            print(f"  {name}: exists (--force to overwrite)"); continue
        print(f"  {name}:")
        ok = process_episode(p, out, marker_info, args)
        print(f"    → {name}\n" if ok else "    SKIPPED\n")

    print("Done.")

if __name__ == '__main__':
    main()