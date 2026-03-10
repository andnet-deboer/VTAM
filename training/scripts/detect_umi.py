#!/usr/bin/env python3
"""
detect_umi.py — Offline AprilTag-based UMI pose detection from episode MCAP bags.

Pipeline position:
    chunk_bag.py → data/chunked/episode_XXX.mcap
                        ↓
                   detect_umi.py
                        ↓
                   data/processed/episode_XXX.mcap  (all original + detection topics)
                        ↓
                   process_demo.py

Reads each episode mcap, runs AprilTag detection on every camera frame,
and writes a new processed mcap that contains:
    - All original messages (verbatim passthrough)
    - /umi_cube_pose       (PoseStamped)  — filtered disconnect pose in base_link
    - /umi_gripper_pose    (PoseStamped)  — 242mm projected gripper in base_link
    - /umi_trajectory      (nav_msgs/Path) — accumulated path at each sync pulse

Usage:
    python3 detect_umi.py pickup_cup
    python3 detect_umi.py pickup_cup --episode 3
    python3 detect_umi.py pickup_cup --force
"""

import argparse
import glob
import os
import sys

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R_scipy
from tqdm import tqdm

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_ros2.reader import read_ros2_messages

# ── Configuration ──────────────────────────────────────────────────────────────

MARKER_YAML = os.path.expanduser(
    "~/VTAM/install/vtam_core/share/vtam_core/config/teleop_april_marker_info_86mm.yaml"
)

SYNC_TOPIC     = "/sync_pulse"
IMAGE_TOPIC    = "/camera/color/image_raw/compressed"
CAM_INFO_TOPIC = "/camera/color/camera_info"
TF_TOPIC       = "/tf"

CAM_TF_PARENT = "base_link"
CAM_TF_CHILD  = "camera_color_optical_frame"

GRIPPER_OFFSET = np.array([0.242, 0.0, 0.0])

# ── CDR serialization helpers ──────────────────────────────────────────────────

CDR_HEADER = b'\x00\x01\x00\x00'


def stamp_to_fields(t_sec):
    sec = int(t_sec)
    nsec = int((t_sec - sec) * 1e9)
    return sec, nsec


def encode_pose_stamped(t_sec, frame_id, pos, quat):
    """Encode geometry_msgs/PoseStamped as CDR bytes."""
    sec, nsec = stamp_to_fields(t_sec)
    frame_bytes = frame_id.encode('utf-8') + b'\x00'
    frame_len = len(frame_bytes)

    buf = bytearray(CDR_HEADER)
    buf += sec.to_bytes(4, 'little', signed=True)
    buf += nsec.to_bytes(4, 'little', signed=False)
    buf += frame_len.to_bytes(4, 'little', signed=False)
    buf += frame_bytes
    while len(buf) % 8 != 0:
        buf += b'\x00'
    for v in pos:
        buf += np.float64(v).tobytes()
    for v in quat:
        buf += np.float64(v).tobytes()
    return bytes(buf)


def encode_path(t_sec, frame_id, poses):
    """Encode nav_msgs/Path as CDR bytes."""
    sec, nsec = stamp_to_fields(t_sec)
    frame_bytes = frame_id.encode('utf-8') + b'\x00'
    frame_len = len(frame_bytes)

    buf = bytearray(CDR_HEADER)
    buf += sec.to_bytes(4, 'little', signed=True)
    buf += nsec.to_bytes(4, 'little', signed=False)
    buf += frame_len.to_bytes(4, 'little', signed=False)
    buf += frame_bytes
    while len(buf) % 4 != 0:
        buf += b'\x00'
    buf += len(poses).to_bytes(4, 'little', signed=False)
    for p_t, p_pos, p_quat in poses:
        while len(buf) % 4 != 0:
            buf += b'\x00'
        p_sec, p_nsec = stamp_to_fields(p_t)
        buf += p_sec.to_bytes(4, 'little', signed=True)
        buf += p_nsec.to_bytes(4, 'little', signed=False)
        empty_frame = b'\x00'
        buf += len(empty_frame).to_bytes(4, 'little', signed=False)
        buf += empty_frame
        while len(buf) % 8 != 0:
            buf += b'\x00'
        for v in p_pos:
            buf += np.float64(v).tobytes()
        for v in p_quat:
            buf += np.float64(v).tobytes()
    return bytes(buf)


# ── Reused detection math (unchanged from UmiDetectorNode) ─────────────────────

class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * np.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x
        dx = (x - self.x_prev) * self.freq
        edx = self.dx_prev + self._alpha(self.d_cutoff) * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        alpha = self._alpha(cutoff)
        x_filtered = self.x_prev + alpha * (x - self.x_prev)
        self.x_prev = x_filtered
        self.dx_prev = edx
        return x_filtered


class AprilTagMarker:
    def __init__(self, tag_id, marker_info):
        self.tag_id = tag_id
        self.info = marker_info.get(str(tag_id), {'length_mm': 64.0, 'frames': {}})
        self.length_mm = self.info.get('length_mm', 64.0)
        self.pos = None
        self.axes = [None, None, None]

    def update(self, corners, camera_matrix, dist_coeffs):
        half = self.length_mm / 2.0
        points_3D = np.array([
            [-half, half, 0], [half, half, 0],
            [half, -half, 0], [-half, -half, 0]
        ])
        _, rvec, tvec = cv2.solvePnP(points_3D, corners, camera_matrix, dist_coeffs)
        self.pos = tvec.flatten() / 1000.0
        R_mat = cv2.Rodrigues(rvec)[0]
        self.axes = [-R_mat[:3, 1], -R_mat[:3, 0], -R_mat[:3, 2]]


def get_cube_pose_from_tag(tag_pos, tag_x, tag_y, tag_z, trans_offset, quat_tag_to_cube):
    cube_pos = (tag_pos
                + trans_offset[0] * tag_x
                + trans_offset[1] * tag_y
                + trans_offset[2] * tag_z)
    R_tag_in_world = np.column_stack((tag_x, tag_y, tag_z))
    R_tag_to_cube = R_scipy.from_quat(quat_tag_to_cube).as_matrix()
    R_cube_in_world = R_tag_in_world @ R_tag_to_cube.T
    return {
        'pos': cube_pos,
        'quat': R_scipy.from_matrix(R_cube_in_world).as_quat()
    }


def calculate_tag_weight(corners):
    pts = corners.reshape(4, 2)
    area = cv2.contourArea(pts)
    return np.clip(area / 1000.0, 0.1, 1.0)


def average_quaternions(quaternions, weights):
    if len(quaternions) == 0:
        return np.array([0, 0, 0, 1])
    quaternions = [np.array(q) / np.linalg.norm(q) for q in quaternions]
    q_ref = quaternions[0]
    aligned_quats = [q if np.dot(q_ref, q) >= 0 else -q for q in quaternions]
    Q = np.zeros((4, 4))
    for q, w in zip(aligned_quats, weights):
        Q += w * np.outer(q, q)
    return np.linalg.eigh(Q)[1][:, -1]


# ── TF helpers ─────────────────────────────────────────────────────────────────

def build_tf_table(tf_data, tf_ts):
    order = np.argsort(tf_ts)
    return np.array(tf_data)[order], np.array(tf_ts)[order]


def lookup_tf(sorted_data, sorted_ts, query_t):
    idx = np.clip(np.searchsorted(sorted_ts, query_t) - 1, 0, len(sorted_data) - 1)
    return sorted_data[idx]


# ── ROS 2 message schemas (for mcap registration) ─────────────────────────────

POSE_STAMPED_SCHEMA = b"""\
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
"""

PATH_SCHEMA = b"""\
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
    builtin_interfaces/Time stamp
      int32 sec
      uint32 nanosec
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
"""


# ── Core detection ─────────────────────────────────────────────────────────────

def detect_episode(mcap_path, marker_info, args):
    """
    Run detection on every camera frame.
    Returns dict with detections list and sync timestamps, or None on failure.
    """

    # -- Pass 1: Read all data --
    camera_matrix = None
    dist_coeffs = None
    sync_ts_ns = []
    img_buf = []
    tf_raw = []
    tf_ts_raw = []

    all_topics = [SYNC_TOPIC, IMAGE_TOPIC, CAM_INFO_TOPIC, TF_TOPIC]

    for msg in read_ros2_messages(mcap_path, topics=all_topics):
        t_ns = msg.publish_time_ns
        t = t_ns / 1e9
        topic = msg.channel.topic

        if topic == SYNC_TOPIC:
            stamp = msg.ros_msg.header.stamp
            sync_ts_ns.append(stamp.sec * 10**9 + stamp.nanosec)

        elif topic == CAM_INFO_TOPIC and camera_matrix is None:
            camera_matrix = np.array(msg.ros_msg.k).reshape((3, 3))
            dist_coeffs = np.array(msg.ros_msg.d)

        elif topic == IMAGE_TOPIC:
            img_buf.append((msg.ros_msg.data, t_ns, t))

        elif topic == TF_TOPIC:
            for tf in msg.ros_msg.transforms:
                if (tf.header.frame_id == CAM_TF_PARENT
                        and tf.child_frame_id == CAM_TF_CHILD):
                    tr = tf.transform.translation
                    ro = tf.transform.rotation
                    tf_raw.append([tr.x, tr.y, tr.z, ro.x, ro.y, ro.z, ro.w])
                    tf_ts_raw.append(t)

    if camera_matrix is None:
        print(f"    No camera_info found, skipping")
        return None
    if len(sync_ts_ns) < 30:
        print(f"    Only {len(sync_ts_ns)} sync pulses, skipping")
        return None
    if not tf_raw:
        print(f"    No TF for {CAM_TF_PARENT} → {CAM_TF_CHILD}, skipping")
        return None

    tf_sorted, tf_ts_sorted = build_tf_table(tf_raw, tf_ts_raw)

    # -- Detector setup --
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    detector = aruco.ArucoDetector(dictionary, params)

    pos_filter = OneEuroFilter(30.0, min_cutoff=args.pos_min_cutoff, beta=args.pos_beta)
    quat_filter = OneEuroFilter(30.0, min_cutoff=args.quat_min_cutoff, beta=args.quat_beta)

    # -- Pass 2: Detect --
    collection = {}
    prev_cube_pose = None
    detections = []  # None or (t_ns, disconnect_pos, disconnect_quat, gripper_pos)
    n_valid = 0

    for compressed_data, frame_t_ns, frame_t in tqdm(img_buf, desc="    Detecting", leave=False):
        cv_image = cv2.imdecode(np.frombuffer(compressed_data, np.uint8), cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            detections.append(None)
            continue

        cube_candidates = []
        for c, aid in zip(corners, ids.flatten()):
            aid = int(aid)
            if aid not in collection:
                collection[aid] = AprilTagMarker(aid, marker_info)
            marker = collection[aid]
            marker.update(c[0], camera_matrix, dist_coeffs)
            frames = marker.info.get('frames', {})
            if 'umi_cube' in frames:
                config = frames['umi_cube']
                res = get_cube_pose_from_tag(
                    marker.pos, marker.axes[0], marker.axes[1], marker.axes[2],
                    config.get('trans', [0.0, 0.0, 0.0]),
                    config.get('quat', [0.0, 0.0, 0.0, 1.0])
                )
                cube_candidates.append({
                    'pos': res['pos'], 'quat': res['quat'],
                    'weight': calculate_tag_weight(c[0])
                })

        if not cube_candidates:
            detections.append(None)
            continue

        weights = np.array([c['weight'] for c in cube_candidates])
        w_norm = weights / weights.sum()
        f_pos_cam = np.average([c['pos'] for c in cube_candidates], axis=0, weights=w_norm)
        f_quat_cam = average_quaternions([c['quat'] for c in cube_candidates], w_norm)

        # TF: camera → base_link
        tf_pose = lookup_tf(tf_sorted, tf_ts_sorted, frame_t)
        R_base_cam = R_scipy.from_quat(tf_pose[3:7])
        T_base_cam = tf_pose[:3]
        fused_pos = T_base_cam + R_base_cam.apply(f_pos_cam)

        cam_rpy = R_base_cam.as_euler('xyz', degrees=True)
        live_pitch = cam_rpy[1]
        R_cube_world = R_base_cam * R_scipy.from_quat(f_quat_cam)
        R_fix = R_scipy.from_euler('zyx', [0, -live_pitch, 180], degrees=True)
        fused_quat = (R_cube_world * R_fix).as_quat()

        # Filtering
        fused_pos = pos_filter.filter(fused_pos)
        if prev_cube_pose is not None and np.dot(fused_quat, prev_cube_pose['quat']) < 0:
            fused_quat = -fused_quat
        fq_raw = quat_filter.filter(fused_quat)
        fused_quat = fq_raw / np.linalg.norm(fq_raw)

        # Z-axis 90° correction
        R_correction = R_scipy.from_euler('z', 90, degrees=True)
        fused_quat_corrected = (R_scipy.from_quat(fused_quat) * R_correction).as_quat()

        prev_cube_pose = {'pos': fused_pos, 'quat': fused_quat}

        # Gripper projection
        R_disc = R_scipy.from_quat(fused_quat_corrected)
        gripper_pos = fused_pos + R_disc.apply(GRIPPER_OFFSET)

        detections.append((frame_t_ns, fused_pos.copy(), fused_quat_corrected.copy(), gripper_pos.copy()))
        n_valid += 1

    pct = 100 * n_valid / max(len(img_buf), 1)
    print(f"    {n_valid}/{len(img_buf)} frames with detections ({pct:.0f}%)")

    return {
        'detections': detections,
        'sync_ts_ns': sync_ts_ns,
        'n_valid': n_valid,
        'n_frames': len(img_buf),
    }


# ── Write processed mcap ──────────────────────────────────────────────────────

def write_processed_mcap(input_path, output_path, detection_result):
    """Copy all original messages and inject detection topics."""
    detections = detection_result['detections']
    sync_set = set(detection_result['sync_ts_ns'])

    # timestamp_ns → detection lookup
    det_by_ts = {}
    for d in detections:
        if d is not None:
            det_by_ts[d[0]] = d

    # Trajectory accumulator
    trajectory_poses = []

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)
        writer.start()

        # Register original channels
        schema_map = {}
        channel_map = {}

        summary = reader.get_summary()
        if summary is None:
            print("    ERROR: mcap has no summary")
            return False

        for sid, schema in summary.schemas.items():
            schema_map[sid] = writer.register_schema(
                name=schema.name, encoding=schema.encoding, data=schema.data
            )
        for cid, channel in summary.channels.items():
            channel_map[cid] = writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_map.get(channel.schema_id, 0),
                metadata=dict(channel.metadata),
            )

        # Register injected channels
        pose_sid = writer.register_schema(
            name="geometry_msgs/msg/PoseStamped",
            encoding="ros2msg", data=POSE_STAMPED_SCHEMA,
        )
        path_sid = writer.register_schema(
            name="nav_msgs/msg/Path",
            encoding="ros2msg", data=PATH_SCHEMA,
        )
        cube_ch = writer.register_channel(
            topic="/umi_cube_pose", message_encoding="cdr", schema_id=pose_sid,
        )
        gripper_ch = writer.register_channel(
            topic="/umi_gripper_pose", message_encoding="cdr", schema_id=pose_sid,
        )
        traj_ch = writer.register_channel(
            topic="/umi_trajectory", message_encoding="cdr", schema_id=path_sid,
        )

        # Stream: passthrough + inject
        for schema, channel, message in reader.iter_messages():
            ts = message.publish_time

            # Passthrough original
            new_ch = channel_map.get(channel.id)
            if new_ch is not None:
                writer.add_message(
                    channel_id=new_ch, log_time=message.log_time,
                    data=message.data, publish_time=ts,
                )

            # Inject poses at camera frames with detections
            if channel.topic == IMAGE_TOPIC and ts in det_by_ts:
                _, d_pos, d_quat, g_pos = det_by_ts[ts]
                t_sec = ts / 1e9

                writer.add_message(
                    channel_id=cube_ch, log_time=ts,
                    data=encode_pose_stamped(t_sec, "base_link", d_pos, d_quat),
                    publish_time=ts,
                )
                writer.add_message(
                    channel_id=gripper_ch, log_time=ts,
                    data=encode_pose_stamped(t_sec, "base_link", g_pos, d_quat),
                    publish_time=ts,
                )
                trajectory_poses.append((t_sec, d_pos.copy(), d_quat.copy()))

            # Emit trajectory at each sync pulse
            if channel.topic == SYNC_TOPIC and ts in sync_set and trajectory_poses:
                writer.add_message(
                    channel_id=traj_ch, log_time=ts,
                    data=encode_path(ts / 1e9, "base_link", trajectory_poses),
                    publish_time=ts,
                )

        writer.finish()
    return True


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Offline UMI pose detection from chunked episode bags."
    )
    parser.add_argument('demo_name', type=str, help="Demo name (e.g. pickup_cup)")
    parser.add_argument('--episode', type=int, default=None,
                        help="Process single episode index")
    parser.add_argument('--force', action='store_true',
                        help="Overwrite existing output files")
    parser.add_argument('--pos-min-cutoff', type=float, default=0.1)
    parser.add_argument('--pos-beta', type=float, default=3.0)
    parser.add_argument('--quat-min-cutoff', type=float, default=0.1)
    parser.add_argument('--quat-beta', type=float, default=3.0)
    args = parser.parse_args()

    # Load marker info
    try:
        with open(MARKER_YAML, 'r') as f:
            marker_info = yaml.safe_load(f)
        print(f"Loaded marker info from: {MARKER_YAML}")
    except Exception as e:
        print(f"ERROR: Could not load marker YAML: {e}")
        sys.exit(1)

    # Paths
    chunked_dir = os.path.expanduser(f"~/VTAM/data/chunked/{args.demo_name}")
    output_dir  = os.path.expanduser(f"~/VTAM/data/processed/{args.demo_name}")
    os.makedirs(output_dir, exist_ok=True)

    if args.episode is not None:
        mcap_paths = [os.path.join(chunked_dir, f"episode_{args.episode:03d}.mcap")]
    else:
        mcap_paths = sorted(glob.glob(os.path.join(chunked_dir, "episode_*.mcap")))

    if not mcap_paths:
        print(f"No episodes found in {chunked_dir}")
        sys.exit(1)

    print(f"Processing {len(mcap_paths)} episode(s)")
    print(f"  Input:  {chunked_dir}")
    print(f"  Output: {output_dir}\n")

    for mcap_path in mcap_paths:
        if not os.path.exists(mcap_path):
            print(f"  {mcap_path} not found, skipping")
            continue

        basename = os.path.basename(mcap_path)
        output_path = os.path.join(output_dir, basename)

        if os.path.exists(output_path) and not args.force:
            print(f"  {basename}: exists, skipping (use --force)")
            continue

        print(f"  {basename}:")
        result = detect_episode(mcap_path, marker_info, args)

        if result is None:
            print(f"    FAILED\n")
            continue
        if result['n_valid'] < 30:
            print(f"    Only {result['n_valid']} valid detections, skipping\n")
            continue

        print(f"    Writing processed mcap...")
        ok = write_processed_mcap(mcap_path, output_path, result)
        if ok:
            print(f"    → {basename}\n")
        else:
            print(f"    WRITE FAILED\n")

    print("Done.")


if __name__ == '__main__':
    main()