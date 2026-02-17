#!/usr/bin/env python3
import argparse
import sys
import os
import time
import numpy as np
import zarr
import logging

# Suppress MuJoCo's internal logging
os.environ['MUJOCO_GL'] = 'egl' 
logging.getLogger('mujoco').setLevel(logging.ERROR)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter

JOINT_NAMES = [
    'joint_mobile_base_rotation', 'joint_mobile_base_translation', 'joint_lift', 'joint_arm_l0',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
]

MUJOCO_JOINT_MAP = {
    'joint_mobile_base_translation': 'base_translate', 
    'joint_mobile_base_rotation': 'base_rotate',
    'joint_lift': 'lift',
    'joint_arm_l0': 'arm',
    'joint_wrist_yaw': 'wrist_yaw',
    'joint_wrist_pitch': 'wrist_pitch',
    'joint_wrist_roll': 'wrist_roll',
}

def load_ee_pose(zarr_path):
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    n_frames = ee_pose.shape[0]
    positions, quaternions = ee_pose[:, :3], ee_pose[:, 3:]
    timestamps = np.arange(n_frames) / 10.0
    print(f"Loaded {zarr_path} ({n_frames} frames)")
    return positions, quaternions, timestamps

def replay_in_mujoco(result, speed=1.0):
    try:
        from stretch_mujoco import StretchMujocoSimulator
    except ImportError:
        print("\nERROR: stretch_mujoco not installed.")
        sys.exit(1)
    
    joints = result['joint_states']
    pos_err = result['pos_error']
    n_frames = joints.shape[0]
    sync_hz = 10.0
    dt = 1.0 / (sync_hz * speed)
    
    sim = StretchMujocoSimulator()
    sim.start()
    time.sleep(2.0)

    # Identify indices for command mapping
    base_rot_idx = JOINT_NAMES.index('joint_mobile_base_rotation')
    base_trans_idx = JOINT_NAMES.index('joint_mobile_base_translation')
    
    try:
        while True:
            print("\n" + "="*40 + "\n  READY: Press [ENTER] to play...")
            input()
            
            for i in range(n_frames):
                loop_start = time.time()
                
                # 1. COMMAND ARM & WRIST (Direct Position Control)
                # We skip the base joints here and handle them via velocity
                for j, name in enumerate(JOINT_NAMES):
                    if 'base' not in name:
                        mj_name = MUJOCO_JOINT_MAP[name]
                        sim.move_to(mj_name, float(joints[i, j]))
                
                # 2. COMMAND BASE (Velocity Tracking)
                try:
                    curr_x, curr_y, curr_yaw = sim.get_base_pose()
                    
                    # Target states from PROJECTED IK result
                    # These start at 0.25 (trans) and 0.0 (yaw)
                    target_trans = joints[i, base_trans_idx]
                    target_yaw   = joints[i, base_rot_idx]
                    
                    # Angular velocity control
                    yaw_err = (target_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
                    omega = yaw_err * sync_hz * speed 

                    # Linear velocity control (Forward/Backward)
                    trans_err = target_trans - curr_x 
                    v_lin = trans_err * sync_hz * speed

                    sim.set_base_velocity(v_linear=float(v_lin), v_angular=float(omega))
                except Exception as e:
                    print(f"Sim Error: {e}")
                
                # Timing control
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)
            
            sim.set_base_velocity(0, 0)
            print(f"\n\n  Playback complete.")
            
    except KeyboardInterrupt:
        print("\n\n  Exiting.")
    finally:
        sim.stop()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zarr', type=str, required=True)
    parser.add_argument('--speed', type=float, default=0.25)
    args = parser.parse_args()
    
    pos, quat, ts = load_ee_pose(args.zarr)
    
    # The Retargeter applies project_to_workspace internally
    # Frame 0 will be forced to [0.25, 0.0, 0.89]
    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(pos, quat, ts)
    
    replay_in_mujoco(result, speed=args.speed)

if __name__ == '__main__':
    main()