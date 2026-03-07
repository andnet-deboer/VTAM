#!/usr/bin/env python3
"""
inference.py — ACT policy inference on Stretch 3.

Requires:
    - stretch_ai ZmqServer running on robot (their server.py)
    - GPU machine running this script

On robot run:
    python3 -m stretch.app.run_server

On GPU machine run:
    python3 inference.py --checkpoint /path/to/checkpoints/060000 --robot_ip 192.168.x.x
"""

import argparse
import sys
import os
import time
import cv2
import numpy as np
import torch

from pathlib import Path
from scipy.spatial.transform import Rotation
from safetensors.torch import load_file

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR    = os.path.join(SCRIPT_DIR, 'training', 'utils')
LEROBOT_DIR  = os.path.join(SCRIPT_DIR, 'dependencies', 'lerobot')
sys.path.insert(0, UTILS_DIR)
sys.path.insert(0, LEROBOT_DIR)

from differential_ik import TrajectoryRetargeter
from lerobot.common.policies.act.modeling_act import ACTPolicy
import rclpy
from geometry_msgs.msg import Twist
from stretch.agent import RobotClient

# ── Constants ──────────────────────────────────────────────────────────────────
IMAGE_SIZE = (320, 320)
FPS        = 10
# Normalization applied internally by policy.select_action

# HelloStretchIdx positions in joint array from server obs.joint
# Verify these against: from stretch.motion.kinematics import HelloStretchIdx
BASE_X_IDX      = 0
LIFT_IDX        = 1
ARM_IDX         = 2
WRIST_YAW_IDX   = 3
WRIST_PITCH_IDX = 4
WRIST_ROLL_IDX  = 5
GRIPPER_IDX     = 6


def load_policy(checkpoint_dir, device):
    policy = ACTPolicy.from_pretrained(checkpoint_dir)
    policy.eval()
    policy.to(device)
    return policy


def load_stats(checkpoint_dir):
    for p in [
        Path(checkpoint_dir) / 'meta_data' / 'stats.safetensors',
        Path(checkpoint_dir).parents[2] / 'meta_data' / 'stats.safetensors',
    ]:
        if p.exists():
            print(f"Loaded stats from {p}")
            return load_file(str(p))
    raise FileNotFoundError(f"stats.safetensors not found near {checkpoint_dir}")


def process_image(rgb_np):
    """RGB numpy (H,W,3) uint8 → float tensor (3,320,320) in [0,1].
    NO manual normalization — policy.select_action applies it internally
    using stats stored in the pretrained model (matches prepare_image in policy_utils.py).
    """
    img = torch.from_numpy(rgb_np).to(torch.float32) / 255.0
    img = img.permute(2, 0, 1)  # (3, H, W)
    # Center crop to 320x320
    _, h, w = img.shape
    s = min(h, w)
    y0, x0 = (h - s) // 2, (w - s) // 2
    img = img[:, y0:y0+s, x0:x0+s]
    img = torch.nn.functional.interpolate(img.unsqueeze(0), size=IMAGE_SIZE, mode='bilinear', align_corners=False).squeeze(0)
    return img


def joints_to_ik_q(joint_positions):
    """
    Map HelloStretchIdx joint array → 7-DOF IK config.
    IK order: [base_rot, base_trans, lift, arm, wrist_yaw, wrist_pitch, wrist_roll]
    """
    return np.array([
        0.0,
        float(joint_positions[BASE_X_IDX]),
        float(joint_positions[LIFT_IDX]),
        float(joint_positions[ARM_IDX]),
        float(joint_positions[WRIST_YAW_IDX]),
        float(joint_positions[WRIST_PITCH_IDX]),
        float(joint_positions[WRIST_ROLL_IDX]),
    ])


def normalize_state(state_8d, stats=None):
    """No manual normalization — policy.select_action handles it internally.
    Matches prepare_state in policy_utils.py which passes raw values."""
    return state_8d.astype(np.float32)


def run_ik_chunk(retargeter, action_chunk, q_seed):
    """
    action_chunk: (N, 8) EE poses [x,y,z,qx,qy,qz,qw,gripper]
    Returns: joint_states (N, 7), gripper_vals (N,)
    """
    q = q_seed.copy()
    joint_states = []
    for i in range(len(action_chunk)):
        for _ in range(retargeter.max_inner_iters):
            q, pe, _ = retargeter.step_ik(q, action_chunk[i, :3], action_chunk[i, 3:7])
            if pe < retargeter.convergence_thresh:
                break
        joint_states.append(q.copy())
    return np.array(joint_states), action_chunk[:, 7]


def send_chunk(robot, cmd_vel_pub, joint_states, gripper_vals):
    """
    Execute chunk step by step.
    Arm joints via robot.arm_to (stretch_ai ZMQ).
    Base rotation via /stretch/cmd_vel directly — avoids manipulation mode conflict.
    joint_states: (N, 7) — [base_rot, base_trans, lift, arm, wyaw, wpitch, wroll]
    gripper_vals: (N,) in [0,1]
    """
    from geometry_msgs.msg import Twist
    dt = 1.0 / FPS
    for i in range(len(joint_states)):
        q = joint_states[i]

        # arm_to order: [base_x, lift, arm, wrist_yaw, wrist_pitch, wrist_roll]
        arm_cmd = np.array([q[1], q[2], q[3], q[4], q[5], q[6]])
        robot.arm_to(arm_cmd, gripper=float(gripper_vals[i]), blocking=False)

        # Base rotation direct on cmd_vel — no mode conflict
        twist = Twist()
        if i < len(joint_states) - 1:
            d_rot = joint_states[i+1, 0] - q[0]
            twist.angular.z = float(d_rot * FPS)
        cmd_vel_pub.publish(twist)

        time.sleep(dt)

    # Stop base
    cmd_vel_pub.publish(Twist())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--robot_ip',   default='')
    parser.add_argument('--n_chunks',   type=int, default=10)
    parser.add_argument('--device',     default='cuda' if torch.cuda.is_available() else 'cpu')
    args = parser.parse_args()

    print(f"Device     : {args.device}")
    print(f"Checkpoint : {args.checkpoint}")

    policy     = load_policy(args.checkpoint, args.device)
    stats      = load_stats(args.checkpoint)
    retargeter = TrajectoryRetargeter()

    rclpy.init()
    _ros_node = rclpy.create_node('inference_node')
    cmd_vel_pub = _ros_node.create_publisher(Twist, '/stretch/cmd_vel', 10)

    robot = RobotClient(robot_ip=args.robot_ip)
    robot.switch_to_manipulation_mode()
    print("Connected. Robot in manipulation mode.")

    input("\nPress [ENTER] to start...")
    policy.reset()

    for chunk_idx in range(args.n_chunks):
        print(f"\n--- Chunk {chunk_idx+1}/{args.n_chunks} ---")

        obs = robot.get_observation()
        rgb  = obs.rgb
        joints = obs.joint

        q_ik     = joints_to_ik_q(joints)
        ee_pos, ee_rot = retargeter.forward_kinematics(q_ik)
        ee_quat  = Rotation.from_matrix(ee_rot).as_quat()
        gripper  = float(joints[GRIPPER_IDX])
        state_8d = np.concatenate([ee_pos, ee_quat, [gripper]])

        batch = {
            'observation.images.gripper': process_image(rgb).unsqueeze(0).to(args.device),
            'observation.state':          torch.from_numpy(
                                              normalize_state(state_8d, stats)
                                          ).unsqueeze(0).to(args.device),
        }

        with torch.no_grad():
            action_chunk = policy.select_action(batch).squeeze(0).cpu().numpy()
        print(f"  Predicted step 0 pos: {action_chunk[0,:3]}")

        joint_states, gripper_vals = run_ik_chunk(retargeter, action_chunk, q_ik)
        send_chunk(robot, cmd_vel_pub, joint_states, gripper_vals)

        # Re-anchor IK to real robot state
        obs = robot.get_observation()
        q_ik = joints_to_ik_q(obs.joint)

    print("\nDone.")
    robot.stop()


if __name__ == '__main__':
    main()