#!/usr/bin/env python3
"""
differential_ik.py — Jacobian-based trajectory retargeting for Stretch 3.

Theory Reference: Lynch & Park, "Modern Robotics" Ch. 6 (Inverse Kinematics)
    - Section 6.2: Iterative Numerical IK
    - Section 6.2.2: Damped Least-Squares (DLS) IK

Pipeline Context:
    This module is the second stage of a two-stage IK pipeline:
        Stage 1 (Seed):  Analytical IK via kinematics.py on first N frames → median joint config
        Stage 2 (Track): Jacobian-based differential IK tracks frame-to-frame

    Input:  obs/ee_pose from synchronized .zarr  (N, 7) as [x, y, z, qx, qy, qz, qw]
    Output: obs/joint_states                     (N, 6) joint angles

Kinematic Chain (6-DOF):
    joint_mobile_base_rotation  (revolute)  — yaw of the base
    joint_lift                  (prismatic) — vertical translation
    joint_arm_l0                (prismatic) — horizontal extension
    joint_wrist_yaw             (revolute)  — wrist yaw
    joint_wrist_pitch           (revolute)  — wrist pitch
    joint_wrist_roll            (revolute)  — wrist roll

Dependencies:
    pip install numpy scipy pinocchio modern_robotics
    Requires: kinematics.py (analytical solver for seeding)
"""

import numpy as np
from scipy.spatial.transform import Rotation
import pinocchio as pin
import modern_robotics as mr

import sys
import os

# --- Import analytical solver for seeding ---
UTILS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, UTILS_DIR)
from VTAM.training.utils.kinematics import StretchIK


class TrajectoryRetargeter:
    """
    Full pipeline: EE pose sequence to Joint States sequence.
    
    Usage:
        retargeter = TrajectoryRetargeter()
        result = retargeter.retarget(positions, quaternions, timestamps)
        joint_states = result['joint_states']  # (N, 6)
    """

    JOINT_NAMES = [
       'joint_mobile_base_rotation', 'joint_mobile_base_translation', 'joint_lift', 'joint_arm_l0',
        'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
    ]

    # Define joint types
    JOINT_TYPES = ['revolute', 'prismatic', 'prismatic', 'prismatic', 'revolute', 'revolute', 'revolute']

    # Joint limits: [min, max] for each joint
    JOINT_LIMITS = np.array([
            [-np.pi, np.pi],   # base rotation
            [-100.0,   100.0],     # base translation
            [ 0.0,   1.1],     # lift
            [ 0.0,   0.52],    # arm extension
            [-1.39,  4.42],    # wrist yaw
            [-1.57,  0.56],    # wrist pitch
            [-3.14,  3.14],    # wrist roll
        ])
    
    def __init__(self, urdf_path=None):
        """
        Initialize the retargeter.
        
        Args:
            urdf_path: Path to Stretch URDF. If None, auto-detect from VTAM layout.
        """
        self.analytical_solver = StretchIK()
        
        # --- Seeding Parameters ---
        self.seed_frames = 10          # Number of initial frames for median seed
        self.seed_max_failures = 5     # If this many fail, flag the demo

        # --- Differential IK Parameters ---
        self.damping = 0.05            # λ for DLS: J^T(JJ^T + λ²I)^{-1}
        self.max_linear_step = 0.05    # meters per frame clamp
        self.max_angular_step = 0.3    # radians per frame clamp
        self.max_joint_step_rev = 1.0  # max revolute joint displacement per frame (rad)
        self.max_joint_step_pri = 0.25 # max prismatic joint displacement per frame (m)
        self.convergence_thresh = 1e-4 # Cartesian error norm to consider "converged"
        self.max_inner_iters = 50       # sub-steps if step gets clamped

        # --- Joint Weights (higher = less motion) ---
        # 7-DOF Weights: Prioritize Rotation to align the heading
        self.joint_weights = np.array([
            0.6,   # base rotation (Making it cheap to face the target)
            0.9,   # base translation (Slightly expensive)
            1.0,   # lift
            1.5,   # arm extension
            0.3,   # wrist yaw
            0.6,   # wrist pitch
            2.0,   # wrist roll
        ])

        # --- Neutral Configuration (mid-range of all joints) ---
        self.neutral_q = np.array([
            0.0,    # base rotation — centered
            0.0,    # base translation — centered
            0.93,   # lift — tabletop height (from dex_teleop)
            0.05,   # arm extension — nearly retracted, max room to extend
            0.0,    # wrist yaw — centered
            0.0,    # wrist pitch — centered
            0.0,    # wrist roll — centered
        ])

        # --- Fallback Parameters ---
        self.fallback_pos_thresh = 0.05  # meters — if tracking error exceeds this, reset
        self.fallback_ori_thresh = 0.3   # radians
        
        if urdf_path is None:
            urdf_path = self._find_urdf()
        
        self._setup_pinocchio(urdf_path)


    def clamp_to_joint_limits(self, q):
        return np.clip(q, self.JOINT_LIMITS[:, 0], self.JOINT_LIMITS[:, 1])

    def _find_urdf(self):
            """Auto-detect the new 7-DOF Omni URDF from the local utils folder."""
            # Get the directory for for URDF search (same as this script)
            utils_dir = os.path.dirname(os.path.abspath(__file__))
            
            omni_candidate = os.path.join(utils_dir, 'stretch_omni_mobile_ik.urdf')
            if os.path.exists(omni_candidate):
                print(f"  [FOUND] Loading local 7-DOF URDF: {omni_candidate}")
                return omni_candidate
            else:
                raise FileNotFoundError("Could not find Stretch URDF locally or in dependencies.")
    
    def _setup_pinocchio(self, urdf_path):
        """
        Load URDF into pinocchio and identify the EE frame.
        """
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        ee_frame_id = model.getFrameId("link_grasp_center")
        joint_ids = [model.getJointId(name) for name in self.JOINT_NAMES]

        self.model = model
        self.data = data
        self.ee_frame_id = ee_frame_id
        self.joint_ids = joint_ids
        
        # Cache the column indices for Jacobian extraction and q mapping
        self.jac_cols = [model.idx_vs[jid] for jid in self.joint_ids]
        self.q_idxs = [model.idx_qs[jid] for jid in self.joint_ids]
        
        print(f"Loaded URDF: {urdf_path}")
        print(f"  EE Frame: {ee_frame_id} ({model.frames[ee_frame_id].name})")
        print(f"  Joint IDs: {joint_ids}")
        print(f"  Jacobian cols: {self.jac_cols}")
        print(f"  Config idxs: {self.q_idxs}")
        print(f"  Model nq={model.nq}, nv={model.nv}")

        print(f"\n=== JOINT MAPPING DEBUG ===")
        for i, name in enumerate(self.JOINT_NAMES):
            jid = self.joint_ids[i]
            q_idx = self.q_idxs[i]
            print(f"  {i}: {name}")
            print(f"      Joint ID: {jid}, Config idx: {q_idx}")
            if jid < len(self.model.joints):
                joint = self.model.joints[jid]
                print(f"      Type: {joint}")

    def _to_full_q(self, q6):
        """Map our 7 joint values into pinocchio's full configuration vector."""
        q_full = pin.neutral(self.model)
        for i, idx in enumerate(self.q_idxs):
            q_full[idx] = q6[i]
        return q_full

    #  STAGE 1: SEEDING — Analytical IK + Median
    # =====================================================================

    def compute_seed(self, positions, quaternions):
        """
        Solve analytical IK on first N frames, return median joint config.
        
        Args:
            positions:   (N, 3) array, full trajectory (only first seed_frames used)
            quaternions: (N, 4) array XYZW
            
        Returns:
            q_seed: (6,) array — seed joint configuration
            
        Raises:
            ValueError: if too many seed frames fail IK
        """
        frames = []
        for i in range(min(self.seed_frames, len(positions))):
            pos = positions[i]
            quat = quaternions[i]
            cfg = self.analytical_solver.solve_single(pos, quat)

            if cfg is not None:
                # Map analytical 6-DOF solution to 7-DOF chain
                # Set base joints to 0.0 or current if they aren't in the analytical solve
                row = []
                for name in self.JOINT_NAMES:
                    if name in cfg:
                        row.append(cfg[name])
                    else:
                        row.append(0.0) # Default for virtual base joints
                frames.append(row)
            
        if len(frames) < self.seed_frames - self.seed_max_failures:
            raise ValueError(f"Too many seed frames failed IK: {len(frames)}/{self.seed_frames}")

        q_seed = np.median(frames, axis=0)
        print(f"  Computed seed from {len(frames)} frames.")
        return q_seed
        
    def project_to_workspace(self, positions, quaternions):
        """
        Anchors the trajectory so the first frame is exactly at the robot's neutral stance.
        Corrects for axis permutations between the demo sensor and ROS/Stretch conventions.
        """
        # 1. Capture the starting point of the human demonstration
        anchor_pos = positions[0]
        R_anchor = Rotation.from_quat(quaternions[0])
        R_anchor_inv = R_anchor.inv()
        
        # 2. Identify the Robot's physical 'Neutral' fingertip pose
        neutral_pos, neutral_rot = self.forward_kinematics(self.neutral_q)
        R_neutral = Rotation.from_matrix(neutral_rot)

        # 3. Calculate alignment rotation
        # Use 'zyx' (Yaw, Pitch, Roll) to safely extract yaw without axis bleeding
        yaw_demo = R_anchor.as_euler('zyx')[0]
        yaw_robot = R_neutral.as_euler('zyx')[0]
        
        # Base alignment + your working 90-degree correction
        yaw_align = yaw_robot - yaw_demo
        correction_angle = -np.pi / 2 
        R_align = Rotation.from_euler('z', yaw_align + correction_angle)

        local_positions = np.zeros_like(positions)
        local_quaternions = np.zeros_like(quaternions)

        for i in range(len(positions)):
            # --- DISPLACEMENT ---
            # Rotate the path vector and anchor it to the robot's hand
            local_positions[i] = neutral_pos + R_align.apply(positions[i] - anchor_pos)
            
            # --- ORIENTATION ---
            # Find the local rotation change from the demo
            R_rel = Rotation.from_quat(quaternions[i]) * R_anchor_inv
            
            # Change of basis: transform the relative rotation into our new -90 rotated frame
            R_rel_aligned = R_align * R_rel * R_align.inv()
            
            # Apply the synchronized relative rotation to the robot's starting orientation
            local_quaternions[i] = (R_neutral * R_rel_aligned).as_quat()

        # Generate T_transform for reference 
        # (Reminder: DO NOT apply this to the base in MuJoCo if you are plotting local_positions!)
        T_transform = np.eye(4)
        T_transform[:3, :3] = R_align.as_matrix()
        T_transform[:3, 3] = neutral_pos - R_align.apply(anchor_pos)

        return local_positions, local_quaternions, T_transform
    
    #  STAGE 2: DIFFERENTIAL IK — Jacobian-based tracking
    # =====================================================================

    def compute_jacobian(self, q):
        """
        Compute the 6x6 geometric Jacobian at configuration q.
        
        Args:
            q: (6,) array — current joint configuration
            
        Returns:
            J: (6, 6) array — geometric Jacobian
        """
        q_full = self._to_full_q(q)
        pin.computeJointJacobians(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        J_full = pin.getFrameJacobian(self.model, self.data, self.ee_frame_id, 
                                       pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J = J_full[:, self.jac_cols]
        return J

    def forward_kinematics(self, q):
        """
        Compute EE pose at configuration q.
        
        Args:
            q: (6,) array — joint configuration
            
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

    def compute_pose_error(self, current_pos, current_rot, target_pos, target_quat):
        """
        Compute the 6D pose error (twist) between current and target.

        Args:
            current_pos:  (3,) current EE position
            current_rot:  (3,3) current EE rotation matrix
            target_pos:   (3,) target position
            target_quat:  (4,) target quaternion XYZW
            
        Returns:
            error: (6,) array — [dx, dy, dz, wx, wy, wz] (pinocchio convention)
        """
        T_current = mr.RpToTrans(current_rot, current_pos)
        R_target = Rotation.from_quat(target_quat).as_matrix()
        T_target = mr.RpToTrans(R_target, target_pos)

        T_err = T_target @ mr.TransInv(T_current)
        error = mr.se3ToVec(mr.MatrixLog6(T_err))
        # Reorder from MR convention [w, v] to pinocchio convention [v, w]
        return np.concatenate([error[3:], error[:3]])
    
    def _dynamic_weights(self, q_current, target_pos):
        """Lower base weight when target is far, lock it when within reach."""
        ee_pos, _ = self.forward_kinematics(q_current)
        dist = np.linalg.norm(target_pos - ee_pos)
        
        arm_reach = 0.52  # max extension
        w = self.joint_weights.copy()
        if dist > arm_reach * 0.6:
            w[0] = 0.1   # base rotation — very cheap
            w[1] = 0.05   # base translation — very cheap
        else:
            w[0] = 5.0   # lock rotation
            w[1] = 5.0   # lock translation
        return w
    
    def weighted_dls_solve(self, J, e, weights=None):
        """
        Weighted damped least-squares solve.
        
        Computes: dq = W^{-1} J^T (J W^{-1} J^T + λ²I)^{-1} e
        
        Args:
            J: (6, 6) Jacobian
            e: (6,) pose error
            
        Returns:
            dq: (6,) joint displacement
        """
        if weights is None:
            weights = self.joint_weights
        W_inv = np.diag(1.0 / weights)
        JWinv = J @ W_inv
        A = JWinv @ J.T + (self.damping ** 2) * np.eye(6)
        dq = W_inv @ J.T @ np.linalg.solve(A, e)
        return dq

    def step_ik(self, q_current, target_pos, target_quat):
        """
        One step of damped least-squares IK.
        
        Args:
            q_current:   (6,) current joint configuration
            target_pos:  (3,) target EE position
            target_quat: (4,) target EE quaternion XYZW
            
        Returns:
            q_new:     (6,) updated joint configuration
            pos_error: float — position error norm (meters)
            ori_error: float — orientation error norm (radians)
        """
        # FK at current config
        current_pos, current_rot = self.forward_kinematics(q_current)

        # Calculate Pose error
        e = self.compute_pose_error(current_pos, current_rot, target_pos, target_quat)

        pos_error = np.linalg.norm(e[:3])
        ori_error = np.linalg.norm(e[3:])

        # Convergence check
        if np.linalg.norm(e) < self.convergence_thresh:
            return q_current.copy(), pos_error, ori_error

        # Jacobian
        J = self.compute_jacobian(q_current)

        # Weighted DLS solve
        weights = self._dynamic_weights(q_current, target_pos)
        dq = self.weighted_dls_solve(J, e, weights=weights)

        # Clamp
        dq = self.clamp_joint_step(dq)

        # Update
        q_new = q_current + dq

        # Enforce joint limits
        q_new = self.clamp_to_joint_limits(q_new)

        return q_new, pos_error, ori_error

    def clamp_joint_step(self, dq):
        """
        Clamp per-joint displacement to prevent large jumps.
        
        Args:
            dq: (6,) proposed joint displacement
            
        Returns:
            dq_clamped: (6,) clamped displacement
        """
        dq_clamped = dq.copy()
        for i in range(len(dq)):
            if self.JOINT_TYPES[i] == 'revolute':
                dq_clamped[i] = np.clip(dq[i], -self.max_joint_step_rev, self.max_joint_step_rev)
            else:
                dq_clamped[i] = np.clip(dq[i], -self.max_joint_step_pri, self.max_joint_step_pri)
        return dq_clamped


    #  FULL PIPELINE
    # =====================================================================

    def retarget(self, positions, quaternions, timestamps):
        """
        Full retargeting pipeline: EE poses → joint trajectory.
        
        Args:
            positions:   (N, 3) array
            quaternions: (N, 4) array XYZW
            timestamps:  (N,) array
            
        Returns:
            dict with keys:
                'joint_states':    (N, 6) array
                'joint_names':     list of 6 strings
                'pos_error':       (N,) array — per-frame position error (m)
                'ori_error':       (N,) array — per-frame orientation error (rad)
                'seed_frames':     int
                'fallback_count':  int
                'timestamps':      (N,) array (pass-through)
                'T_transform':     (4, 4) array — SE(3) workspace projection
                'base_xy':         (2,) array — base position for deployment
        """
        N = len(positions)
        quaternions = self._standardize_quats(quaternions)

        print("\n=== RETARGETING DEBUG ===")
        print(f"Raw position[0]: {positions[0]}")

        # 1. Get the Median Z height from the first few demo frames directly (No IK needed)
        seed_count = min(self.seed_frames, len(positions))
        raw_median_z = np.median(positions[:seed_count, 2])
        
        # 2. Set the neutral lift carriage height (adjusting for the 0.097m gripper offset)
        # This solves the "15cm jump" by anchoring to the demo's actual height
        neutral_q = self.neutral_q.copy()
        lift_idx = self.JOINT_NAMES.index('joint_lift')
        neutral_q[lift_idx] = np.clip(raw_median_z - 0.097, 0.0, 1.1)
        self.neutral_q = neutral_q

        # 3. NOW project the trajectory (this shifts the path so the robot can reach it)
        positions, quaternions, T_transform = self.project_to_workspace(positions, quaternions)

        # 4. NOW compute the seed (this will work now because the frames are within reach)
        q_seed = self.compute_seed(positions, quaternions)
        
        print(f"\nAfter workspace projection:")
        print(f"  Projected position[0]: {positions[0]}")
        print(f"  Neutral config (Anchor): {self.neutral_q}")

        # Stage 1: Seed with analytical IK median
        q_seed = self.neutral_q.copy()
        seed_count = 0  # no analytical seeding used

        # Stage 2: Track with differential IK
        n_joints = len(self.JOINT_NAMES)
        joint_states = np.zeros((N, n_joints))
        pos_errors = np.zeros(N)
        ori_errors = np.zeros(N)
        fallback_count = 0

        q = q_seed.copy()

        for i in range(N):
            # Inner iteration loop — multiple sub-steps if clamping limits progress
            for _ in range(self.max_inner_iters):
                q_new, pe, oe = self.step_ik(q, positions[i], quaternions[i])

                # Check if converged
                if pe < self.convergence_thresh and oe < self.convergence_thresh:
                    break
                q = q_new

            # Final error evaluation at the actual output config
            q = q_new
            fk_pos, fk_rot = self.forward_kinematics(q)
            e_final = self.compute_pose_error(fk_pos, fk_rot, positions[i], quaternions[i])
            pos_errors[i] = np.linalg.norm(e_final[:3])
            ori_errors[i] = np.linalg.norm(e_final[3:])
            joint_states[i] = q

            # DEBUG: Print first frame results
            if i == 0:
                print(f"\n=== FIRST FRAME IK RESULT ===")
                print(f"  Target: {positions[0]}")
                print(f"  Achieved: {fk_pos}")
                print(f"  Joint states: {joint_states[0]}")
                print(f"  Base rotation: {joint_states[0, 0]:.3f} rad ({np.degrees(joint_states[0, 0]):.1f} deg)")
                print(f"  Base translation: {joint_states[0, 1]:.3f} m")
                print(f"  Lift: {joint_states[0, 2]:.3f} m")
                print(f"  Arm: {joint_states[0, 3]:.3f} m")
                print(f"  Position error: {pos_errors[0]*1000:.1f} mm")

            # Fallback check — if tracking diverged, try analytical reset
            if pos_errors[i] > self.fallback_pos_thresh or ori_errors[i] > self.fallback_ori_thresh:
                cfg = self.analytical_solver.solve_single(positions[i], quaternions[i])
                if cfg is not None:
                    q = np.array([cfg.get(n, 0.0) for n in self.JOINT_NAMES])
                    fk_pos, fk_rot = self.forward_kinematics(q)
                    e = self.compute_pose_error(fk_pos, fk_rot, positions[i], quaternions[i])
                    pos_errors[i] = np.linalg.norm(e[:3])
                    ori_errors[i] = np.linalg.norm(e[3:])
                    joint_states[i] = q
                fallback_count += 1

        return {
            'joint_states': joint_states,
            'joint_names': list(self.JOINT_NAMES),
            'pos_error': pos_errors,
            'ori_error': ori_errors,
            'seed_frames': seed_count,
            'fallback_count': fallback_count,
            'timestamps': timestamps,
            'T_transform': T_transform,
            'base_xy': T_transform[:2, 3],
            'target_positions': positions,      
            'target_quaternions': quaternions,
        }
    
    def _standardize_quats(self, quats):
        """Flip quaternions to ensure shortest rotation path between consecutive frames."""
        quats = quats.copy()
        for i in range(1, len(quats)):
            if np.dot(quats[i - 1], quats[i]) < 0:
                quats[i] = -quats[i]
        return quats


#  STANDALONE TEST
# =========================================================================

if __name__ == '__main__':
    print("=== Differential IK Self-Test ===\n")
    
    retargeter = TrajectoryRetargeter()
    
    N = 50
    t = np.linspace(0, 5, N)
    positions = np.column_stack([
        -0.1 * np.ones(N),
        -0.4 + 0.1 * np.sin(t),
        0.6 * np.ones(N),
    ])
    
    base_quat = Rotation.from_euler('y', 0.5).as_quat()
    quaternions = np.tile(base_quat, (N, 1))
    
    result = retargeter.retarget(positions, quaternions, t)
    
    print(f"Seed frames used:    {result['seed_frames']}")
    print(f"Fallback resets:     {result['fallback_count']}")
    print(f"Mean position error: {np.mean(result['pos_error'])*1000:.2f} mm")
    print(f"Max position error:  {np.max(result['pos_error'])*1000:.2f} mm")
    print(f"Mean orient. error:  {np.degrees(np.mean(result['ori_error'])):.2f} deg")
    print(f"Max orient. error:   {np.degrees(np.max(result['ori_error'])):.2f} deg")