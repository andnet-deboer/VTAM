#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import zarr
import matplotlib.pyplot as plt

# --- Path Integration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Note: Ensure this points to where your updated differential_ik.py lives
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter

def run_diagnostic(zarr_path):
    """Retargets and plots results with dedicated mobile base analysis."""
    # 1. Load Demo
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    pos = ee_pose[:, :3] # [X, Y, Z]
    quat = ee_pose[:, 3:]
    
    # 2. Compute IK (Now using the 8-DOF Omni URDF)
    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(pos, quat, np.arange(len(pos))/10.0)
    
    joints = result['joint_states']
    joint_names = result['joint_names']
    
    # 3. Create Three-Pane Diagnostic Plot
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    
    # TOP PANE: Target Cartesian Motion
    ax1.plot(pos[:, 0], label='Target X (Forward)', color='red', alpha=0.8)
    ax1.plot(pos[:, 1], label='Target Y (Left)', color='green', alpha=0.8)
    ax1.plot(pos[:, 2], label='Target Z (Up)', color='blue', alpha=0.8)
    ax1.set_title(f"Target EE Trajectory (Projected)\nDemo: {os.path.basename(zarr_path)}")
    ax1.set_ylabel("Meters")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # MIDDLE PANE: Mobile Base Coordination (The new 8-DOF logic)
    # Finding indices dynamically to avoid IndexErrors if names change
    try:
        rot_idx = joint_names.index('joint_mobile_base_rotation')
        trans_idx = joint_names.index('joint_mobile_base_translation')
        ax2.plot(joints[:, rot_idx], label='Base Rotation (rad)', color='purple', linewidth=2)
        ax2.plot(joints[:, trans_idx], label='Base Translation (m)', color='orange', linewidth=2)
        ax2.set_title("Mobile Base: Rotation vs. Translation")
        ax2.set_ylabel("Rad / Meters")
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
    except ValueError:
        print("[WARNING] Mobile base joint names not found in result.")

    # BOTTOM PANE: Manipulation Joints (Lift, Arm, Wrists)
    for i in range(joints.shape[1]):
        if 'base' not in joint_names[i]: # Focus on the arm/wrist
            ax3.plot(joints[:, i], label=joint_names[i])
    
    # Add a horizontal line for the physical arm limit (0.52m)
    ax3.axhline(y=0.52, color='black', linestyle='--', alpha=0.5, label='Physical Arm Limit')
    
    ax3.set_title("IK Result: Arm & Wrist Activity")
    ax3.set_xlabel("Frame Index")
    ax3.set_ylabel("Meters / Radians")
    ax3.legend(loc='upper right', ncol=2)
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print(f"\n[SUCCESS] 8-DOF Diagnostic generated for {os.path.basename(zarr_path)}")
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--zarr', required=True)
    args = parser.parse_args()
    run_diagnostic(args.zarr)