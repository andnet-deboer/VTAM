#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import zarr
from scipy.spatial.transform import Rotation as R

# Patching NumPy
if not hasattr(np, "bool"):
    np.bool = bool

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores

# --- PATH CONFIG ---
SIMPLE_IK_DIR = os.path.expanduser("~/VTAM/dependencies/stretch_dex_teleop")
sys.path.insert(0, SIMPLE_IK_DIR)
import simple_ik as si

# 7-DOF State: Driving the base while manipulating
JOINT_NAMES = [
    "joint_mobile_base_rotation", "joint_lift", "joint_arm_l0",
    "joint_wrist_yaw", "joint_wrist_pitch", "joint_wrist_roll",
    "joint_mobile_base_translation"
]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=str, required=True)
    parser.add_argument("--output", type=str, required=True)
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # Initialize IK
    orig_dir = os.getcwd()
    try:
        os.chdir(os.path.abspath(SIMPLE_IK_DIR))
        ik_solver = si.SimpleIK()
    finally:
        os.chdir(orig_dir)

    GRIPPER_OFFSET_X = 0.242 # From your detector node
    
    print(f"Full Mobile-Base Trajectory IK: {args.bag}")
    
    data = {'ts': [], 'pos': [], 'quat': []}
    with Reader(args.bag) as reader:
        conns = [x for x in reader.connections if x.topic == '/umi_cube_pose']
        for conn, _, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.position, msg.pose.orientation
            data['ts'].append(ts)
            data['pos'].append([p.x, p.y, p.z])
            data['quat'].append([q.x, q.y, q.z, q.w])

    valid_j, valid_ts = [], []

    for i in range(len(data['ts'])):
        ts = data['ts'][i]
        T_base_handle = np.eye(4)
        T_base_handle[:3, 3] = data['pos'][i]
        T_base_handle[:3, :3] = R.from_quat(data['quat'][i]).as_matrix()
        
        # Project target to Gripper Center
        T_base_gripper = T_base_handle @ np.array([[1,0,0,GRIPPER_OFFSET_X],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        target_pos = T_base_gripper[:3, 3]
        target_rot = T_base_gripper[:3, :3]

        # 1. Manually calculate required base rotation to face the target XY
        base_theta = np.arctan2(target_pos[1], target_pos[0])

        # 2. Use Prismatic Base IK to find Translation, Lift, and Extension
        res_ik = ik_solver.ik_prismatic_base(target_pos)
        
        if res_ik:
            r = R.from_matrix(target_rot)
            ypr = r.as_euler("ZXY", degrees=False)
            
            # Map the returned keys correctly. SimpleIK prismatic keys are:
            # 'joint_mobile_base_translation', 'joint_lift', 'joint_arm_l0'
            state = [
                float(base_theta),                                    # Manually calculated
                float(res_ik['joint_lift']),
                float(res_ik['joint_arm_l0']),
                float((ypr[0] + np.pi + np.pi) % (2 * np.pi) - np.pi), # Yaw
                float(ypr[1]),                                        # Pitch
                float((ypr[2] + np.pi + np.pi) % (2 * np.pi) - np.pi), # Roll
                float(res_ik['joint_mobile_base_translation'])        # Base driving
            ]
            valid_j.append(state)
            valid_ts.append(ts)

    if len(valid_j) < 5:
        print("Error: IK Failed. Target likely outside driving/lift limits.")
        return

    # Save to Zarr
    store = zarr.DirectoryStore(args.output)
    root = zarr.group(store=store, overwrite=True)
    dgrp = root.create_group("data")
    dgrp.create_dataset("state", data=np.array(valid_j[:-1], dtype=np.float32))
    dgrp.create_dataset("action", data=np.array(valid_j[1:], dtype=np.float32))
    dgrp.create_dataset("timestamp", data=np.array(valid_ts[:-1], dtype=np.float64))
    
    meta = root.create_group("meta")
    meta.create_dataset("episode_ends", data=np.array([len(valid_j)-1], dtype=np.int64))
    root.attrs["fps"] = 1.0 / np.mean(np.diff(valid_ts))
    root.attrs["joint_names"] = JOINT_NAMES
    
    print(f"Success! {len(valid_j)-1} driving frames saved to {args.output}")

if __name__ == "__main__":
    main()