#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import zarr
import matplotlib.pyplot as plt

# --- Path Integration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter

def run_diagnostic(zarr_path):
    """Visualizes retargeting results by calling the TrajectoryRetargeter API."""
    
    # 1. Data Ingestion
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    pos_raw = ee_pose[:, :3] 
    quat_raw = ee_pose[:, 3:]
    timestamps = np.arange(len(pos_raw)) / 10.0
    
    # 2. Call Implementation Class
    # The class handles coordinate projection and IK internally
    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(pos_raw, quat_raw, timestamps)
    
    # Retrieve projected trajectory for workspace visualization
    pos_proj, _, T_trans = retargeter.project_to_workspace(pos_raw, quat_raw)
    
    joints = result['joint_states']
    joint_names = result['joint_names']
    
    # 3. Diagnostic Plotting
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 14), sharex=True)
    
    # TOP PANE: Workspace Alignment
    ax1.plot(pos_proj[:, 0], label='Projected X (Forward)', color='red', linewidth=2)
    ax1.plot(pos_proj[:, 1], label='Projected Y (Left)', color='green', linewidth=2)
    ax1.plot(pos_proj[:, 2], label='Projected Z (Up)', color='blue', linewidth=2)
    
    # Visual baselines for Northwestern MSR neutral stance
    ax1.axhline(y=0.25, color='red', linestyle=':', alpha=0.5, label='Neutral X Target')
    ax1.axhline(y=0.89, color='blue', linestyle=':', alpha=0.5, label='Neutral Z Target')
    
    ax1.set_title(f"Projected EE Trajectory (Relative to Robot Origin)\nDemo: {os.path.basename(zarr_path)}")
    ax1.set_ylabel("Meters")
    ax1.legend(loc='upper right', ncol=2)
    ax1.grid(True, alpha=0.3)

    # MIDDLE PANE: Mobile Base Coordination (8-DOF Analysis)
    try:
        rot_idx = joint_names.index('joint_mobile_base_rotation')
        trans_idx = joint_names.index('joint_mobile_base_translation')
        ax2.plot(joints[:, rot_idx], label='Base Rotation (rad)', color='purple', linewidth=2)
        ax2.plot(joints[:, trans_idx], label='Base Translation (m)', color='orange', linewidth=2)
        ax2.set_title("Mobile Base: Coordination Tracking")
        ax2.set_ylabel("Rad / Meters")
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
    except ValueError:
        print("[WARNING] Expected base joint names missing from retargeter output.")

    # BOTTOM PANE: Manipulation Joints (Lift, Arm, Wrist)
    for i in range(joints.shape[1]):
        if 'base' not in joint_names[i]:
            ax3.plot(joints[:, i], label=joint_names[i])
    
    # Physical limit for Hello Robot Stretch telescopic arm
    ax3.axhline(y=0.52, color='black', linestyle='--', alpha=0.5, label='Physical Arm Limit')
    
    ax3.set_title("IK Tracking Result: Arm & Wrist Activity")
    ax3.set_xlabel("Frame Index")
    ax3.set_ylabel("Meters / Radians")
    ax3.legend(loc='upper right', ncol=2)
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Terminal Summary
    print(f"\n[DIAGNOSTIC SUMMARY]")
    print(f"  Frame 0 Projected Position: {pos_proj[0]}")
    print(f"  Total Projection Offset:    {T_trans[:3, 3]}")
    print(f"  Mean Tracking Error:        {np.mean(result['pos_error'])*1000:.2f} mm")
    
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--zarr', required=True, help="Path to processed .zarr demo file")
    args = parser.parse_args()
    run_diagnostic(args.zarr)