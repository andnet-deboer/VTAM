#!/usr/bin/env python3
"""
fuse_imu.py — Offline IMU + ArUco pose fusion for VTAM demonstration bags.

Reads an episode MCAP bag, fuses /umi_imu/raw + /umi_cube_pose using
GTSAM ISAM2 IMU preintegration, and writes a new bag with an additional
/umi_fused_pose topic at IMU rate (~500Hz) with gaps filled.

Usage:
    python fuse_imu.py --input episode_001.mcap --output episode_001_fused.mcap

    # Override topics if needed:
    python fuse_imu.py --input episode_001.mcap --output episode_001_fused.mcap \
        --imu_topic /umi_imu/raw --pose_topic /umi_cube_pose

Pipeline position:
    bag_chunker.py → episode_001.mcap → fuse_imu.py → episode_001_fused.mcap → process_demo.py
"""

import argparse
import sys
import os
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

# ── Add utils/ to path so we can import gtsam_fusion_core ────────────────────
sys.path.insert(0, str(Path(__file__).parent.parent / 'utils'))
from gtsam_fusion_core import GtsamFusionCore, numpy_pose_to_gtsam, gtsam_pose_to_numpy

# ── MCAP / ROS2 bag reader ────────────────────────────────────────────────────
try:
    from rosbags.rosbag2 import Reader, Writer
    from rosbags.typesys import Stores, get_typestore
    from rosbags.typesys.stores.ros2_humble import (
        sensor_msgs__msg__Imu as Imu,
        geometry_msgs__msg__PoseStamped as PoseStamped,
        geometry_msgs__msg__PoseWithCovarianceStamped as PoseWithCovStamped,
    )
except ImportError:
    print("ERROR: rosbags not installed. Run: pip install rosbags")
    sys.exit(1)

# ── ISM330DLC noise parameters (tune after allan variance analysis) ───────────
# These are conservative starting values — tighten after calibration
ISM330DLC_PARAMS = {
    # ISAM2 optimizer
    'relinearize_th': 0.1,
    'relinearize_skip': 10,
    'factorization': 'CHOLESKY',
    'opt_meas_buffer_time': 0.05,   # 50ms buffer to handle out-of-order msgs

    # Gravity vector (ENU frame: g points down = negative Z)
    'g': [0.0, 0.0, -9.81],

    # ISM330DLC datasheet noise densities
    'sigma_accelerometer': 0.003,   # m/s²/√Hz — tighten after allan variance
    'sigma_gyroscope':     0.0002,  # rad/s/√Hz
    'sigma_integration':   1e-4,

    # Earth rotation (negligible for indoor short demos)
    'use_2nd_order_coriolis': False,
    'omega_coriolis': [0.0, 0.0, 0.0],

    # Body-to-sensor transform — CHANGE THIS to match your physical mount
    # Default: IMU aligned with gripper frame (no rotation, no offset)
    'b2s_pos': [0.0, 0.0, 0.0],
    'b2s_ori': [0.0, 0.0, 0.0, 1.0],  # xyzw

    # Initial state — will be overridden by first ArUco detection
    'init_pos': [0.0, 0.0, 0.0],
    'init_ori': [0.0, 0.0, 0.0, 1.0],  # xyzw
    'init_vel': [0.0, 0.0, 0.0],
    'init_acc_bias': [0.0, 0.0, 0.0],
    'init_gyr_bias': [0.0, 0.0, 0.0],

    # Initial uncertainty (large = trust first ArUco heavily)
    'sigma_init_pos': 0.5,
    'sigma_init_ori': 0.5,
    'sigma_init_vel': 0.3,
    'sigma_acc_init_bias': 0.1,
    'sigma_gyr_init_bias': 0.01,

    # ArUco pose measurement noise
    # Tune: tighter = trust ArUco more, looser = trust IMU more
    'sigma_pose_pos': np.array([0.01, 0.01, 0.01]),   # 1cm std
    'sigma_pose_rot': np.array([0.05, 0.05, 0.05]),   # ~3deg std

    # GPS noise (unused, but required by gtsam_fusion_core)
    'sigma_gps': [1.0, 1.0, 1.0],

    # Bias random walk — how fast bias drifts between ArUco updates
    'sigma_acc_bias_evol': np.array([1e-4, 1e-4, 1e-4]),
    'sigma_gyr_bias_evol': np.array([1e-5, 1e-5, 1e-5]),
}


def stamp_to_sec(stamp) -> float:
    """Convert ROS2 timestamp to float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def sec_to_stamp(t: float):
    """Convert float seconds to (sec, nanosec) tuple."""
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return sec, nanosec


def load_messages(bag_path: str, topics: list[str]) -> list[tuple]:
    """
    Read all messages from specified topics, return sorted list of
    (timestamp_sec, topic, msg_dict).
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    messages = []

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        if not connections:
            print(f"ERROR: None of {topics} found in bag.")
            print(f"Available topics: {[c.topic for c in reader.connections]}")
            sys.exit(1)

        for conn, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            t_sec = timestamp * 1e-9  # nanoseconds → seconds
            messages.append((t_sec, conn.topic, msg))

    messages.sort(key=lambda x: x[0])
    print(f"Loaded {len(messages)} messages from {len(connections)} topics.")
    return messages


def run_fusion(messages: list, imu_topic: str, pose_topic: str) -> list[tuple]:
    """
    Run GTSAM ISAM2 fusion over sorted message list.
    Returns list of (timestamp_sec, position, orientation_xyzw) fused poses.
    """
    # ── Find first ArUco pose to initialize state ─────────────────────────────
    init_pos, init_ori = None, None
    for t, topic, msg in messages:
        if topic == pose_topic:
            p = msg.pose.position
            o = msg.pose.orientation
            init_pos = np.array([p.x, p.y, p.z])
            init_ori = np.array([o.x, o.y, o.z, o.w])
            print(f"Initializing from first ArUco at t={t:.3f}s  pos={init_pos}")
            break

    if init_pos is None:
        print("ERROR: No ArUco pose messages found. Cannot initialize fusion.")
        sys.exit(1)

    params = dict(ISM330DLC_PARAMS)
    params['init_pos'] = init_pos.tolist()
    params['init_ori'] = init_ori.tolist()

    fusion = GtsamFusionCore(params)

    fused_poses = []
    prev_imu_time = None
    imu_count = 0
    pose_count = 0

    print("Running fusion...")
    for t, topic, msg in messages:
        if topic == imu_topic:
            a = msg.linear_acceleration
            g = msg.angular_velocity
            linear_acc = np.array([a.x, a.y, a.z])
            angular_vel = np.array([g.x, g.y, g.z])

            # Compute dt
            if prev_imu_time is None:
                dt = 1.0 / 500.0  # assume 500Hz for first sample
            else:
                dt = t - prev_imu_time
                if dt <= 0 or dt > 0.1:
                    # Skip bad dt (packet loss, clock jump)
                    prev_imu_time = t
                    continue
            prev_imu_time = t

            result = fusion.add_imu_measurement(t, linear_acc, angular_vel, dt)
            if result is not None:
                pos, ori, vel, acc_bias, gyr_bias = result
                fused_poses.append((t, pos, ori))
            imu_count += 1

        elif topic == pose_topic:
            p = msg.pose.position
            o = msg.pose.orientation
            position = np.array([p.x, p.y, p.z])
            orientation = np.array([o.x, o.y, o.z, o.w])

            # Use absolute pose (ArUco gives global position in camera frame)
            fusion.add_absolute_pose_measurement(t, position, orientation)
            pose_count += 1

    print(f"Processed {imu_count} IMU samples, {pose_count} ArUco corrections.")
    print(f"Generated {len(fused_poses)} fused pose estimates.")
    return fused_poses


def write_output_bag(input_path: str, output_path: str,
                     fused_poses: list, fused_topic: str,
                     imu_topic: str, pose_topic: str):
    """
    Copy input bag to output, adding /umi_fused_pose alongside existing topics.
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    with Reader(input_path) as reader:
        with Writer(output_path) as writer:
            # ── Copy all existing connections ─────────────────────────────────
            conn_map = {}
            for conn in reader.connections:
                new_conn = writer.add_connection(
                    conn.topic, conn.msgtype,
                    typestore=typestore
                )
                conn_map[conn.id] = new_conn

            # ── Add fused pose connection ─────────────────────────────────────
            fused_conn = writer.add_connection(
                fused_topic,
                'geometry_msgs/msg/PoseStamped',
                typestore=typestore
            )

            # ── Copy all original messages ────────────────────────────────────
            for conn, timestamp, rawdata in reader.messages():
                writer.write(conn_map[conn.id], timestamp, rawdata)

            # ── Write fused poses ─────────────────────────────────────────────
            for t_sec, pos, ori in fused_poses:
                msg = PoseStamped()
                msg.header.frame_id = 'base_link'
                msg.header.stamp.sec, msg.header.stamp.nanosec = sec_to_stamp(t_sec)
                msg.pose.position.x = float(pos[0])
                msg.pose.position.y = float(pos[1])
                msg.pose.position.z = float(pos[2])
                msg.pose.orientation.x = float(ori[0])
                msg.pose.orientation.y = float(ori[1])
                msg.pose.orientation.z = float(ori[2])
                msg.pose.orientation.w = float(ori[3])

                timestamp_ns = int(t_sec * 1e9)
                rawdata = typestore.serialize_cdr(msg, 'geometry_msgs/msg/PoseStamped')
                writer.write(fused_conn, timestamp_ns, rawdata)

    print(f"Output bag written: {output_path}")
    print(f"New topic '{fused_topic}' added with {len(fused_poses)} messages.")


def main():
    parser = argparse.ArgumentParser(description='Offline IMU + ArUco pose fusion.')
    parser.add_argument('--input',      required=True,  help='Input MCAP bag path')
    parser.add_argument('--output',     required=True,  help='Output MCAP bag path')
    parser.add_argument('--imu_topic',  default='/umi_imu/raw',   help='IMU topic name')
    parser.add_argument('--pose_topic', default='/umi_cube_pose', help='ArUco pose topic name')
    parser.add_argument('--fused_topic',default='/umi_fused_pose',help='Output fused pose topic')
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"ERROR: Input bag not found: {args.input}")
        sys.exit(1)

    print(f"Input:       {args.input}")
    print(f"Output:      {args.output}")
    print(f"IMU topic:   {args.imu_topic}")
    print(f"Pose topic:  {args.pose_topic}")
    print(f"Fused topic: {args.fused_topic}")
    print()

    messages   = load_messages(args.input, [args.imu_topic, args.pose_topic])
    fused_poses = run_fusion(messages, args.imu_topic, args.pose_topic)
    write_output_bag(args.input, args.output, fused_poses,
                     args.fused_topic, args.imu_topic, args.pose_topic)

    print("\nDone. Next step: process_demo.py --input", args.output)


if __name__ == '__main__':
    main()