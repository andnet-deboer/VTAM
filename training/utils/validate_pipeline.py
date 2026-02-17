#!/usr/bin/env python3
import argparse
import sys
import os
import numpy as np
import zarr
import matplotlib.pyplot as plt

# python3 validate_pipeline.py --zarr ../../data/processed/demo/demo_20260216_020448_0.zarr

# --- Path Integration ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)

from differential_ik import TrajectoryRetargeter

def run_diagnostic(zarr_path):
    """Retargets and plots results using standard 2D axes to avoid 3D import errors."""
    # 1. Load Demo
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    pos = ee_pose[:, :3] # [X, Y, Z]
    quat = ee_pose[:, 3:]
    
    # 2. Compute IK
    retargeter = TrajectoryRetargeter()
    # Forces Stage 0 projection and Stage 2 Differential IK tracking
    result = retargeter.retarget(pos, quat, np.arange(len(pos))/10.0)
    
    joints = result['joint_states']
    joint_names = result['joint_names']
    
    # 3. Create Multi-Pane Diagnostic Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    # TOP PANE: Target Cartesian Motion (X, Y, Z)
    ax1.plot(pos[:, 0], label='Target X (Forward)', color='red')
    ax1.plot(pos[:, 1], label='Target Y (Left)', color='green')
    ax1.plot(pos[:, 2], label='Target Z (Up)', color='blue')
    ax1.set_title(f"Target EE Trajectory (Projected Workspace)\nDemo: {os.path.basename(zarr_path)}")
    ax1.set_ylabel("Meters")
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # BOTTOM PANE: Resulting Joint Activity
    for i in range(joints.shape[1]):
        ax2.plot(joints[:, i], label=joint_names[i], linewidth=2)
    ax2.set_title("IK Result: Joint Activity")
    ax2.set_xlabel("Frame Index")
    ax2.set_ylabel("Meters / Radians")
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    print("\n[SUCCESS] Diagnostic plot generated. Check for cross-talk between axes.")
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--zarr', required=True)
    args = parser.parse_args()
    run_diagnostic(args.zarr)