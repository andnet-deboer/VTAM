#!/usr/bin/env python3
import argparse
import sys
import os
import time
import numpy as np
import zarr
import logging
# Suppress MuJoCo warnings and logging
os.environ['MUJOCO_LOG_LEVEL'] = '0'

logging.getLogger('mujoco').setLevel(logging.ERROR)
logging.getLogger('stretch_mujoco').setLevel(logging.ERROR)

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
    if hasattr(sim, 'viewer'):
        sim.viewer.sync() # Force a sync
    sim.start()
    # Visualize target trajectory
    retarget_positions = result.get('target_positions')  # we need to pass this through
    if retarget_positions is not None:
        step = max(1, len(retarget_positions) // 30)  # ~30 markers
        for i in range(0, len(retarget_positions), step):
            p = retarget_positions[i]
            sim.add_world_frame(position=(float(p[0]), float(p[1]), float(p[2])))
            print(f"  Drew {len(range(0, len(retarget_positions), step))} trajectory markers")

    # --- Pre-align MuJoCo to Frame 0 ---
    print("\nPre-aligning robot to the first trajectory frame...")
    for j, name in enumerate(JOINT_NAMES):
        if 'base' not in name:
            mj_name = MUJOCO_JOINT_MAP[name]
            sim.move_to(mj_name, float(joints[0, j]))
            
    # Give the simulated controllers 1 second to physically move the arm into place
    time.sleep(1.0) 
    # -----------------------------------

    # Identify indices for command mapping
    base_rot_idx = JOINT_NAMES.index('joint_mobile_base_rotation')
    base_trans_idx = JOINT_NAMES.index('joint_mobile_base_translation')

    try:
        while True:
            print("\n" + "="*40 + "\n  READY: Press [ENTER] to play...")
            input()

            
            for i in range(n_frames):
                loop_start = time.time()
                
                # Move Arm and Wrist
                # We skip the base joints here and handle them via velocity
                for j, name in enumerate(JOINT_NAMES):
                    if 'base' not in name:
                        mj_name = MUJOCO_JOINT_MAP[name]
                        sim.move_to(mj_name, float(joints[i, j]))
                
                # Move Base
                try:
                    curr_x, curr_y, curr_yaw = sim.get_base_pose()
                    
                    target_trans = joints[i, base_trans_idx]
                    target_yaw   = joints[i, base_rot_idx]
                    
                    # Angular velocity
                    yaw_err = (target_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
                    w = yaw_err * sync_hz * speed 

                    # Linear velocity — project world pos onto robot heading
                    curr_forward = curr_x * np.cos(curr_yaw) + curr_y * np.sin(curr_yaw)
                    trans_err = target_trans - curr_forward
                    v = trans_err * sync_hz * speed

                    sim.set_base_velocity(v_linear=float(v), omega=float(w))
                    
                    if i % 10 == 0:
                        print(f"[Frame {i:4d}] "
                              f"tgt_trans={target_trans:+.4f}  curr_fwd={curr_forward:+.4f}  v_lin={v:+.4f} | "
                              f"tgt_yaw={target_yaw:+.4f}  curr_yaw={curr_yaw:+.4f}  omega={w:+.4f}")
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
    parser.add_argument('--speed', type=float, default=1.0)
    args = parser.parse_args()
    
    pos, quat, ts = load_ee_pose(args.zarr)
    
    # The Retargeter applies project_to_workspace internally
    # Frame 0 will be forced to [0.25, 0.0, 0.89]
    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(pos, quat, ts)
    
    replay_in_mujoco(result, speed=args.speed)

if __name__ == '__main__':
    main()