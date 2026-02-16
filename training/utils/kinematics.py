#!/usr/bin/env python3
"""
kinematics.py — Offline IK for trajectory retargeting.

This module provides 6-DOF Inverse Kinematics for the Hello Robot Stretch 3.
It handles the translation between Cartesian goals and joint states, 
accounting for the base rotation and the dexterous wrist decomposition.
"""

import sys
import os
import numpy as np
import math
from contextlib import contextmanager
from scipy.spatial.transform import Rotation

# Environment Constants
STRETCH_DEX_DIR = os.path.expanduser('~/VTAM/dependencies/stretch_dex_teleop')
sys.path.insert(0, STRETCH_DEX_DIR)

# Import Hello Robot dependency
try:
    import simple_ik as si
except ImportError:
    print(f"Error: Could not find simple_ik in {STRETCH_DEX_DIR}")
    sys.exit(1)

@contextmanager
def working_directory(path):
    """Safely switch working directory to load localized URDF assets."""
    origin = os.getcwd()
    try:
        os.chdir(path)
        yield
    finally:
        os.chdir(origin)

def angle_diff_rad(target, current):
    """Calculate the smallest signed angle difference in [-π, π]."""
    diff = target - current
    return ((diff + math.pi) % (2.0 * math.pi)) - math.pi

class StretchIK:
    def __init__(self):
        """
        Initialize the IK solver. 
        Note: simple_ik expects specialized URDFs in the working directory.
        """
        with working_directory(STRETCH_DEX_DIR):
            # Attempt to initialize with defaults. 
            # If your simple_ik requires positional args, add them here.
            self.ik = si.SimpleIK()

        self.joint_names = [
            'joint_mobile_base_rotation', 'joint_lift', 'joint_arm_l0',
            'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
        ]

        # Stretch 3 Dexterous Wrist Limits (Standard Revolute)
        self.wrist_limits = {
            'joint_wrist_yaw': (-1.75, 4.0),
            'joint_wrist_pitch': (-1.57, 0.56),
            'joint_wrist_roll': (-3.14, 3.14)
        }

        # Stability Parameters
        self.max_yaw_change = 1.5
        self.max_roll_change = 1.5
        self.pos_alpha = 0.1
        self.ori_alpha = 0.5

    def decompose_orientation(self, rotation_matrix, debug=False):
        r = Rotation.from_matrix(rotation_matrix)
        ypr = r.as_euler('ZXY', degrees=False)

        # Stretch 3 offsets
        wrist_yaw = angle_diff_rad(ypr[0] + np.pi, 0.0)
        wrist_pitch = ypr[1]
        wrist_roll = angle_diff_rad(ypr[2] + np.pi, 0.0)

        if wrist_yaw < self.wrist_limits['joint_wrist_yaw'][0]:
            wrist_yaw += 2.0 * np.pi
            
        # Detailed Debugging
        joints = [
            ('joint_wrist_yaw', wrist_yaw),
            ('joint_wrist_pitch', wrist_pitch),
            ('joint_wrist_roll', wrist_roll)
        ]

        for name, val in joints:
            lo, hi = self.wrist_limits[name]
            if not (lo <= val <= hi):
                if debug:
                    print(f"  [WRIST FAIL] {name}: {val:.3f} is outside [{lo}, {hi}]")
                return None

        return wrist_yaw, wrist_pitch, wrist_roll

    def solve_single(self, position, quat_xyzw, debug=False):
        # 1. Position IK
        pos_cfg = self.ik.ik_rotary_base(position)
        if pos_cfg is None:
            if debug: print(f"  [POS FAIL] Position {position} unreachable.")
            return None

        # 2. Orientation IK
        rot_mat = Rotation.from_quat(quat_xyzw).as_matrix()
        wrist = self.decompose_orientation(rot_mat, debug=debug)
        if wrist is None:
            return None

        pos_cfg.update({
            'joint_wrist_yaw': wrist[0],
            'joint_wrist_pitch': wrist[1],
            'joint_wrist_roll': wrist[2]
        })
        return pos_cfg

    def solve_single(self, position, quat_xyzw, debug=False):
        """
        Solve 6-DOF IK for one pose relative to base_link.
        """
        # 1. Position IK (Base, Lift, Arm)
        pos_cfg = self.ik.ik_rotary_base(position)
        if pos_cfg is None:
            if debug: print(f"DEBUG: Position unreachable: {position}")
            return None

        # 2. Orientation IK (Wrist)
        rot_mat = Rotation.from_quat(quat_xyzw).as_matrix()
        wrist = self.decompose_orientation(rot_mat)
        if wrist is None:
            if debug: print(f"DEBUG: Orientation out of limits.")
            return None

        pos_cfg.update({
            'joint_wrist_yaw': wrist[0],
            'joint_wrist_pitch': wrist[1],
            'joint_wrist_roll': wrist[2]
        })
        return pos_cfg

    def retarget_trajectory(self, positions, quaternions, timestamps, hz=10.0):
        """
        Convert a pose sequence into a smooth, time-synced joint trajectory.
        """
        positions = np.array(positions)
        quats = self._standardize_quats(np.array(quaternions))
        
        raw_joints = []
        prev_wrist = None

        for i in range(len(positions)):
            cfg = self.solve_single(positions[i], quats[i])
            
            if cfg is None:
                raw_joints.append(raw_joints[-1] if raw_joints else None)
                continue

            # Continuity Check
            if prev_wrist is not None:
                if (abs(cfg['joint_wrist_yaw'] - prev_wrist[0]) > self.max_yaw_change or 
                    abs(cfg['joint_wrist_roll'] - prev_wrist[2]) > self.max_roll_change):
                    raw_joints.append(raw_joints[-1])
                    continue

            vec = np.array([cfg[n] for n in self.joint_names])
            raw_joints.append(vec)
            prev_wrist = (cfg['joint_wrist_yaw'], cfg['joint_wrist_pitch'], cfg['joint_wrist_roll'])

        return self._process_raw_trajectory(raw_joints, timestamps, hz)

    def _standardize_quats(self, quats):
        """Flip quaternions to ensure the shortest rotation path."""
        for i in range(1, len(quats)):
            if np.dot(quats[i-1], quats[i]) < 0:
                quats[i] = -quats[i]
        return quats

    def _process_raw_trajectory(self, raw, ts, hz):
        """Resamples and applies exponential smoothing."""
        valid = [i for i, x in enumerate(raw) if x is not None]
        if len(valid) < 2:
            raise ValueError("IK failed for too many frames.")

        traj = np.array([raw[i] for i in valid])
        ts_valid = ts[valid]

        new_ts = np.arange(ts_valid[0], ts_valid[-1], 1.0 / hz)
        resampled = np.zeros((len(new_ts), len(self.joint_names)))
        for j in range(len(self.joint_names)):
            resampled[:, j] = np.interp(new_ts, ts_valid, traj[:, j])

        for i in range(1, len(resampled)):
            for j, name in enumerate(self.joint_names):
                alpha = self.ori_alpha if 'wrist' in name else self.pos_alpha
                resampled[i, j] = (1 - alpha) * resampled[i-1, j] + alpha * resampled[i, j]

        return {'timestamps': new_ts, 'joints': resampled, 'names': self.joint_names}


def draw_ascii_workspace(position, success):
        px, py, pz = position
        # Normalize coordinates for a 20x20 grid
        # Grid covers -1.0 to 1.0 meters
        def to_grid(val): return int((val + 1.0) * 10)
        
        grid = [[" " for _ in range(21)] for _ in range(21)]
        
        # Draw Robot Base
        grid[to_grid(0)][to_grid(-0.1)] = "R" 
        
        # Draw Target
        marker = "X" if success else "!"
        if 0 <= to_grid(py) <= 20 and 0 <= to_grid(px) <= 20:
            grid[to_grid(py)][to_grid(px)] = marker

        print("\n--- TOP DOWN VIEW (X: Front/Back, Y: Left/Right) ---")
        print("      [Front +1.0m]")
        for row in reversed(grid):
            print("|" + "".join(row) + "|")
        print("      [Back -1.0m]")
        print(f"Target Status: {'REACHABLE' if success else 'OUT OF BOUNDS'}")

# Usage in your __main__:
# draw_ascii_workspace(test_pos, True)
if __name__ == '__main__':
    solver = StretchIK()
    
    # The 'Identity' quaternion [0,0,0,1] means the gripper is pointing 
    # forward in the world. On a Stretch, if the arm is extended, 
    # this orientation is often physically impossible because of the Pitch limit.
    
    print("\n--- IK DIAGNOSTIC REPORT ---")
    test_pos = [-0.1, -0.4, 0.6] 
    test_quat = [0, 0, 0, 1] 
    
    # Try a slightly 'pitched down' orientation which is the Stretch's sweet spot
    down_quat = Rotation.from_euler('y', 0.5).as_quat() 

    print(f"Testing Position: {test_pos}")
    print("Testing Identity Orientation...")
    res1 = solver.solve_single(test_pos, test_quat, debug=True)
    
    print("\nTesting Pitched-Down Orientation (more likely to succeed)...")
    res2 = solver.solve_single(test_pos, down_quat, debug=True)

    if res2:
        print("\nSUCCESS found with modified orientation!")
    draw_ascii_workspace(test_pos, True)