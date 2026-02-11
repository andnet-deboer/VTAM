#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np

# Patching NumPy for compatibility with older trimesh/urchin versions
if not hasattr(np, "bool"):
    np.bool = bool

import zarr
from scipy.spatial.transform import Rotation
from pathlib import Path

# ROS 2 Imports
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

# rosbags library for parsing MCAP
from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

# --- PATH CONFIG ---
SIMPLE_IK_DIR = os.path.expanduser("~/VTAM/dependencies/stretch_dex_teleop")
sys.path.insert(0, SIMPLE_IK_DIR)
import simple_ik as si

# --- GLOBALS ---
TARGET_FRAME = "umi_cube"
ROBOT_BASE_FRAME = "base_link"
JOINT_NAMES = [
    "joint_mobile_base_rotation", "joint_lift", "joint_arm_l0",
    "joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll",
    "stretch_gripper",
]

def solve_ik_for_pose(trans, rot_matrix, ik):
    """Industry-standard IK solver wrapper for Stretch mobile base."""
    config = ik.ik_rotary_base(trans)
    if config is None:
        return None
    
    r = Rotation.from_matrix(rot_matrix)
    ypr = r.as_euler("ZXY", degrees=False)
    
    def angle_diff(a, b):
        diff = a - b
        while diff > np.pi: diff -= 2*np.pi
        while diff < -np.pi: diff += 2*np.pi
        return diff
    
    wrist_yaw = angle_diff(ypr[0] + np.pi, 0.0)
    wrist_pitch = ypr[1]
    wrist_roll = angle_diff(ypr[2] + np.pi, 0.0)
    
    return [
        config["joint_mobile_base_rotation"], config["joint_lift"],
        config["joint_arm_l0"], wrist_yaw, wrist_pitch, wrist_roll, 0.0
    ]

def load_tf_buffer_from_bag(bag_path):
    """Methodical loading: Converts all rosbags data to native ROS 2 types."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    tf_buffer = Buffer(Duration(seconds=500))
    master_timestamps = []

    print(f"Parsing bag: {bag_path}")
    with Reader(bag_path) as reader:
        connections = [x for x in reader.connections if x.topic in ['/tf', '/tf_static']]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            is_static = (connection.topic == '/tf_static')
            
            # Store timestamps from dynamic /tf to act as the Master Clock
            if not is_static:
                master_timestamps.append(timestamp)

            for t in msg.transforms:
                official_t = TransformStamped()
                official_t.header.frame_id = t.header.frame_id
                official_t.child_frame_id = t.child_frame_id
                
                # Make statics time-invariant to bridge the base_link -> link_mast gap
                if is_static:
                    official_t.header.stamp = Time(nanoseconds=0).to_msg()
                else:
                    official_t.header.stamp.sec = t.header.stamp.sec
                    official_t.header.stamp.nanosec = t.header.stamp.nanosec
                
                official_t.transform.translation = Vector3(
                    x=float(t.transform.translation.x), 
                    y=float(t.transform.translation.y), 
                    z=float(t.transform.translation.z)
                )
                official_t.transform.rotation = Quaternion(
                    x=float(t.transform.rotation.x), 
                    y=float(t.transform.rotation.y), 
                    z=float(t.transform.rotation.z), 
                    w=float(t.transform.rotation.w)
                )
                
                tf_buffer.set_transform(official_t, "static" if is_static else "bag")

    return tf_buffer, sorted(list(set(master_timestamps)))

def process_bag(bag_path, ik):
    """Synchronized Resampling: Solving at the exact heartbeats of recorded /tf data."""
    tf_buffer, master_timestamps = load_tf_buffer_from_bag(bag_path)
    if not master_timestamps: return None, None

    valid_joints, valid_ts = [], []
    print(f"Attempting to solve IK for {len(master_timestamps)} events...")

    for t_ns in master_timestamps:
        try:
            # PROFESSIONAL FIX: 20ms tolerance bridges the gap between 
            # asynchronous Camera and Robot State updates.
            trans_stamped = tf_buffer.lookup_transform(
                ROBOT_BASE_FRAME, 
                TARGET_FRAME, 
                Time(nanoseconds=t_ns), 
                Duration(seconds=0.02)
            )
            
            trans = np.array([
                trans_stamped.transform.translation.x, 
                trans_stamped.transform.translation.y, 
                trans_stamped.transform.translation.z
            ])
            q = trans_stamped.transform.rotation
            rot_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            
            joints = solve_ik_for_pose(trans, rot_matrix, ik)
            if joints:
                valid_joints.append(joints)
                valid_ts.append(t_ns * 1e-9)
                
        except TransformException:
            # Skip if chain is incomplete (e.g., cube not in camera frame).
            continue

    return np.array(valid_ts), np.array(valid_joints)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    args = parser.parse_args()

    rclpy.init() # Must initialize before directory changes

    original_dir = os.getcwd()
    abs_ik_dir = os.path.abspath(SIMPLE_IK_DIR)
    
    try:
        print(f"Moving to {abs_ik_dir} to load specialized URDFs...")
        os.chdir(abs_ik_dir)
        ik = si.SimpleIK()
    except Exception as e:
        print(f"CRITICAL: IK Initialization failed: {e}")
        return
    finally:
        os.chdir(original_dir)

    ts, joints = process_bag(args.bag, ik)
    
    if ts is not None and len(ts) > 0:
        store = zarr.DirectoryStore(args.output)
        root = zarr.group(store=store, overwrite=True)
        data = root.create_group("data")
        data.create_dataset("state", data=joints.astype(np.float32))
        
        # Action is the next state in the sequence for imitation learning
        actions = np.roll(joints, -1, axis=0)
        actions[-1] = joints[-1]
        data.create_dataset("action", data=actions.astype(np.float32))
        data.create_dataset("timestamp", data=ts.astype(np.float64))
        
        meta = root.create_group("meta")
        meta.create_dataset("episode_ends", data=np.array([len(ts)], dtype=np.int64))
        
        print(f"Success! Converted {len(ts)} samples to {args.output}")
    else:
        print("CRITICAL: Failed to stitch chain. Try increasing 'Duration' in lookup_transform.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()