#!/usr/bin/env python3
import argparse
import sys
import os
import time
import numpy as np
import zarr
import logging

# Suppress MuJoCo's internal logging
os.environ['MUJOCO_GL'] = 'egl' # Use EGL if on Linux/No-display, otherwise 'osmesa' or 'glfw'
logging.getLogger('mujoco').setLevel(logging.ERROR)

# Add training/utils to path for the IK modules
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter

JOINT_NAMES = [
    'joint_mobile_base_rotation', 'joint_lift', 'joint_arm_l0',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
]

# Mapping between URDF joint names and MuJoCo names
MUJOCO_JOINT_MAP = {
    'joint_mobile_base_translation': 'base_translate', # Verify this in your XML
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

def print_frame_status(frame_idx, n_frames, joints, pos_err, ori_err):
    short_names = ['base', 'lift', 'arm', 'wyaw', 'wpit', 'wrol']
    joint_str = ' '.join(f'{n}:{v:+.2f}' for n, v in zip(short_names, joints))
    print(f'\r  Frame {frame_idx:4d}/{n_frames} | {joint_str} | err: {pos_err*1000:.1f}mm', end='', flush=True)

def replay_in_mujoco(result, speed=1.0):
    """Cleaned replay using high-level non-blocking commands."""
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

    base_idx = JOINT_NAMES.index('joint_mobile_base_rotation')
    arm_indices = [j for j, name in enumerate(JOINT_NAMES) if name != 'joint_mobile_base_rotation']

    try:
        while True:
            print("\n" + "="*40 + "\n  READY: Press [ENTER] to play...")
            input()
            
            for i in range(n_frames):
                loop_start = time.time()
                
                # 1. COMMAND ARM (Non-blocking by default in this API)
                for j in arm_indices:
                    mj_name = MUJOCO_JOINT_MAP[JOINT_NAMES[j]]
                    # We use the float conversion to be safe
                    sim.move_to(mj_name, float(joints[i, j]))
                
                # 2. COMMAND BASE (Velocity Tracking)
                # Since we can't use move_to for base_rotate, we pull pose and drive
                # 2. Update Velocity Tracking in replay_in_mujoco loop
                try:
                    # Current pose from simulator [x, y, yaw]
                    curr_x, curr_y, curr_yaw = sim.get_base_pose()
                    
                    # Target states from IK result
                    target_trans = joints[i, 1]  # joint_mobile_base_translation
                    target_yaw   = joints[i, 0]  # joint_mobile_base_rotation
                    
                    # Calculate angular velocity (omega)
                    yaw_err = (target_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
                    omega = yaw_err * sync_hz * speed 

                    # Calculate linear velocity (v_linear) 
                    # This assumes base_translation maps to forward/backward movement
                    trans_err = target_trans - curr_x 
                    v_lin = trans_err * sync_hz * speed

                    sim.set_base_velocity(v_linear=float(v_lin), v_angular=float(omega))
                except Exception as e:
                    print(f"Sim Error: {e}")
                
                print_frame_status(i, n_frames, joints[i], pos_err[i], 0)
                
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
    parser.add_argument('--speed', type=float, default=0.25) # Default slow-mo for validation
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    
    pos, quat, ts = load_ee_pose(args.zarr)
    
    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(pos, quat, ts)
    
    if not args.dry_run:
        replay_in_mujoco(result, speed=args.speed)

if __name__ == '__main__':
    main()