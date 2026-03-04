#!/usr/bin/env python3
"""
workspace_projection.py — Workspace projection for Stretch 3.

Pipeline Context:
    This module is the FIRST stage only of the two-stage pipeline in differential_ik.py.
    It stops after project_to_workspace() and does NOT run differential IK.

    Input:  raw EE poses from UMI sensor  (N, 7) as [x, y, z, qx, qy, qz, qw]
    Output: canonical EE poses            (N, 7) in robot frame

    For the full pipeline including differential IK, see differential_ik.py.

Kinematic Chain (7-DOF) — same as differential_ik.py:
    joint_mobile_base_rotation  (revolute)  — yaw of the base
    joint_mobile_base_translation (prismatic) — forward translation
    joint_lift                  (prismatic) — vertical translation
    joint_arm_l0                (prismatic) — horizontal extension
    joint_wrist_yaw             (revolute)  — wrist yaw
    joint_wrist_pitch           (revolute)  — wrist pitch
    joint_wrist_roll            (revolute)  — wrist roll

Dependencies:
    pip install numpy scipy pinocchio
"""

import numpy as np
from scipy.spatial.transform import Rotation
import pinocchio as pin

import sys
import os

UTILS_DIR = os.path.dirname(os.path.abspath(__file__))


class WorkspaceProjector:
    """
    Stage 1 only: project raw UMI hand poses into robot canonical frame.

    Identical to TrajectoryRetargeter.project_to_workspace() in differential_ik.py
    but without the differential IK stage.

    Usage:
        projector = WorkspaceProjector()
        result = projector.project(positions, quaternions, timestamps)
        canonical_poses = result['ee_poses']  # (N, 7) [x,y,z,qx,qy,qz,qw]
    """

    JOINT_NAMES = [
       'joint_mobile_base_rotation', 'joint_mobile_base_translation', 'joint_lift', 'joint_arm_l0',
        'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
    ]

    def __init__(self, urdf_path=None):
        """
        Initialize the projector.

        Args:
            urdf_path: Path to Stretch URDF. If None, auto-detect from VTAM layout.
        """
        # --- Neutral Configuration (mid-range of all joints) ---
        self.neutral_q = np.array([
            0.0,    # base rotation — centered
            0.0,    # base translation — centered
            0.89,   # lift — tabletop height (from dex_teleop)
            0.05,   # arm extension — nearly retracted, max room to extend
            0.0,    # wrist yaw — centered
            0.0,    # wrist pitch — centered
            0.0,    # wrist roll — centered
        ])

        if urdf_path is None:
            urdf_path = self._find_urdf()

        self._setup_pinocchio(urdf_path)

    def _find_urdf(self):
        """Auto-detect the new 7-DOF Omni URDF from the local utils folder."""
        utils_dir = os.path.dirname(os.path.abspath(__file__))

        omni_candidate = os.path.join(utils_dir, 'stretch_omni_mobile_ik.urdf')
        if os.path.exists(omni_candidate):
            print(f"  [FOUND] Loading local 7-DOF URDF: {omni_candidate}")
            return omni_candidate
        else:
            raise FileNotFoundError("Could not find Stretch URDF locally or in dependencies.")

    def _setup_pinocchio(self, urdf_path):
        """Load URDF into pinocchio and identify the EE frame."""
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        ee_frame_id = model.getFrameId("link_grasp_center")
        joint_ids = [model.getJointId(name) for name in self.JOINT_NAMES]

        self.model = model
        self.data = data
        self.ee_frame_id = ee_frame_id
        self.joint_ids = joint_ids

        self.jac_cols = [model.idx_vs[jid] for jid in self.joint_ids]
        self.q_idxs = [model.idx_qs[jid] for jid in self.joint_ids]

        print(f"Loaded URDF: {urdf_path}")
        print(f"  EE Frame: {ee_frame_id} ({model.frames[ee_frame_id].name})")

    def _to_full_q(self, q7):
        """Map our 7 joint values into pinocchio's full configuration vector."""
        q_full = pin.neutral(self.model)
        for i, idx in enumerate(self.q_idxs):
            q_full[idx] = q7[i]
        return q_full

    def forward_kinematics(self, q):
        """
        Compute EE pose at configuration q.

        Args:
            q: (7,) array — joint configuration

        Returns:
            position: (3,) array
            rotation: (3, 3) array
        """
        q_full = self._to_full_q(q)
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.ee_frame_id]
        position = oMf.translation.copy()
        rotation = oMf.rotation.copy()
        return position, rotation

    def project_to_workspace(self, positions, quaternions, neutral_q_override=None):
        """
        Anchors the trajectory so the first frame is exactly at the robot's neutral stance.
        Corrects for axis permutations between the demo sensor and ROS/Stretch conventions.

        Identical to TrajectoryRetargeter.project_to_workspace() in differential_ik.py.
        """
        neutral_q = neutral_q_override if neutral_q_override is not None else self.neutral_q

        anchor_pos = positions[0]
        R_anchor = Rotation.from_quat(quaternions[0])
        R_anchor_inv = R_anchor.inv()

        neutral_pos, neutral_rot = self.forward_kinematics(neutral_q) 
        R_neutral = Rotation.from_matrix(neutral_rot)

        yaw_demo = R_anchor.as_euler('zyx')[0]
        yaw_robot = R_neutral.as_euler('zyx')[0]
        yaw_align = yaw_robot - yaw_demo
        correction_angle = -np.pi / 2
        R_align = Rotation.from_euler('z', yaw_align + correction_angle)

        local_positions = np.zeros_like(positions)
        local_quaternions = np.zeros_like(quaternions)

        for i in range(len(positions)):
            local_positions[i] = neutral_pos + R_align.apply(positions[i] - anchor_pos)
            R_rel = Rotation.from_quat(quaternions[i]) * R_anchor_inv
            R_rel_aligned = R_align * R_rel * R_align.inv()
            local_quaternions[i] = (R_neutral * R_rel_aligned).as_quat()

        T_transform = np.eye(4)
        T_transform[:3, :3] = R_align.as_matrix()
        T_transform[:3, 3] = neutral_pos - R_align.apply(anchor_pos)

        return local_positions, local_quaternions, T_transform

    def _standardize_quats(self, quats):
        """Flip quaternions to ensure shortest rotation path between consecutive frames."""
        quats = quats.copy()
        for i in range(1, len(quats)):
            if np.dot(quats[i - 1], quats[i]) < 0:
                quats[i] = -quats[i]
        return quats

    #  Projection Pipeline 
    # =====================================================================

    def project(self, positions, quaternions, timestamps):
        """
        Project raw UMI hand poses into robot canonical frame.

        Args:
            positions:   (N, 3) array
            quaternions: (N, 4) array XYZW
            timestamps:  (N,) array

        Returns:
            dict with keys:
                'ee_poses':     (N, 7) array — [x,y,z,qx,qy,qz,qw] in canonical frame
                'timestamps':   (N,) array (pass-through)
                'T_transform':  (4, 4) array — SE(3) workspace projection
        """
        N = len(positions)
        quaternions = self._standardize_quats(quaternions)

        print("\n=== PROJECTION DEBUG ===")
        print(f"Raw position[0]: {positions[0]}")

        # Get the Median Z height from the first few demo frames directly
        seed_count = min(10, len(positions))
        raw_median_z = np.median(positions[:seed_count, 2])

        # Set the neutral lift carriage height (adjusting for the 0.097m gripper offset)
        neutral_q = self.neutral_q.copy()
        lift_idx = self.JOINT_NAMES.index('joint_lift')
        neutral_q[lift_idx] = np.clip(raw_median_z - 0.097, 0.0, 1.1)

        positions, quaternions, T_transform = self.project_to_workspace(
            positions, quaternions, neutral_q_override=neutral_q
        )

        print(f"After workspace projection:")
        print(f"  Projected position[0]: {positions[0]}")
        print(f"  Episode lift height: {neutral_q[lift_idx]:.3f}m")

        ee_poses = np.concatenate([positions, quaternions], axis=1)

        return {
            'ee_poses':    ee_poses,
            'timestamps':  timestamps,
            'T_transform': T_transform,
        }
    