#!/usr/bin/env python3
import argparse
import sys
import os
import time
import numpy as np
import zarr
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
UTILS_DIR = os.path.join(SCRIPT_DIR, '..', 'utils')
sys.path.insert(0, UTILS_DIR)
from differential_ik import TrajectoryRetargeter

IK_TO_ROBOT_NAMES = {
    'joint_mobile_base_rotation': 'rotate_mobile_base',
    'joint_mobile_base_translation': 'translate_mobile_base',
    'joint_lift': 'joint_lift',
    'joint_arm_l0': 'joint_arm_l0',
    'joint_wrist_yaw': 'joint_wrist_yaw',
    'joint_wrist_pitch': 'joint_wrist_pitch',
    'joint_wrist_roll': 'joint_wrist_roll',
}

# Joints sent via FollowJointTrajectory (no base, arm split into 4)
ARM_JOINT_NAMES = [
    'joint_lift',
    'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
    'joint_gripper_finger_left', 
]

class RobotReplayNode(Node):
    def __init__(self):
        super().__init__('robot_replay_node')
        self._action_client = ActionClient(
            self, FollowJointTrajectory, 
            '/stretch_controller/follow_joint_trajectory'
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)

    def send_arm_trajectory(self, ik_names, ik_states, fps=10.0, is_prealign=False):
        """Send non-base joints via FollowJointTrajectory, splitting arm into 4."""
        self.get_logger().info('Connecting to /stretch_controller...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return None
        
        # Build index map from IK output
        lift_idx = ik_names.index('joint_lift')
        arm_idx = ik_names.index('joint_arm_l0')
        wyaw_idx = ik_names.index('joint_wrist_yaw')
        wpitch_idx = ik_names.index('joint_wrist_pitch')
        wroll_idx = ik_names.index('joint_wrist_roll')
        grip_idx = ik_names.index('joint_gripper_finger_left') if 'joint_gripper_finger_left' in ik_names else None
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINT_NAMES

        for i, states in enumerate(ik_states):
            point = JointTrajectoryPoint()
            arm_per_joint = float(states[arm_idx]) / 4.0
            point.positions = [
                float(states[lift_idx]),
                arm_per_joint, arm_per_joint, arm_per_joint, arm_per_joint,
                float(states[wyaw_idx]),
                float(states[wpitch_idx]),
                float(states[wroll_idx]),
                float(states[grip_idx]) if grip_idx is not None else 0.0,
            ]
            if is_prealign:
                point.time_from_start.sec = 4
                point.time_from_start.nanosec = 0
            else:
                t_sec = (i + 1) / fps
                point.time_from_start.sec = int(t_sec)
                point.time_from_start.nanosec = int((t_sec % 1) * 1e9)
            goal_msg.trajectory.points.append(point)

        return self._action_client.send_goal_async(goal_msg)

def load_zarr_joints(zarr_path):
    root = zarr.open(zarr_path, mode='r')
    ee_pose = np.array(root['obs/ee_pose'])
    positions, quaternions = ee_pose[:, :3], ee_pose[:, 3:]
    timestamps = np.arange(ee_pose.shape[0]) / 10.0

    retargeter = TrajectoryRetargeter()
    result = retargeter.retarget(positions, quaternions, timestamps)
    
    joint_names = list(result['joint_names'])
    joint_states = result['joint_states']
    
    # Append gripper as pass-through
    if 'obs/gripper_position' in root['obs']:
        gripper = np.array(root['obs/gripper_position'])  # (N,1)
        if gripper.ndim == 1:
            gripper = gripper.reshape(-1, 1)
        joint_states = np.column_stack([joint_states, gripper])
        joint_names.append('joint_gripper_finger_left')
    
    return joint_names, joint_states

def sanity_check(names, states, fps=10.0):
    print("\n--- Pre-flight Sanity Check ---")
    valid = True
    for i, name in enumerate(names):
        j_min, j_max = np.min(states[:, i]), np.max(states[:, i])
        j_vels = np.abs(np.diff(states[:, i])) * fps
        max_v = np.max(j_vels)
        print(f"{name:40} | Range: [{j_min:6.2f}, {j_max:6.2f}] | Max Vel: {max_v:6.2f} rad/s")
        if name == 'joint_lift' and (j_min < 0.1 or j_max > 1.1):
            print(f"  !! ERROR: Lift exceeds safety bounds [0.1, 1.1]")
            valid = False
        if name == 'joint_arm_l0' and (j_min < 0.0 or j_max > 0.52):
            print(f"  !! ERROR: Arm exceeds safety bounds [0.0, 0.52]")
            valid = False
        if max_v > 2.5:
            print(f"  !! WARNING: {name} velocity ({max_v:.2f}) is very high.")
    return valid

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--zarr', type=str, required=True)
    parser.add_argument('--speed', type=float, default=0.3)
    args = parser.parse_args()

    joint_names, joint_states = load_zarr_joints(args.zarr)

    if not sanity_check(joint_names, joint_states):
        print("\nERROR: Demo failed sanity check. Exiting for safety.")
        return

    # Extract base indices
    base_rot_idx = joint_names.index('joint_mobile_base_rotation')
    base_trans_idx = joint_names.index('joint_mobile_base_translation')
    fps = 10.0 * args.speed

    rclpy.init()
    node = RobotReplayNode()

    # --- PRE-ALIGNMENT (arm only) ---
    print(f"\nMove to START POSE?")
    input("Press [ENTER] to confirm...")

    pre_future = node.send_arm_trajectory(joint_names, [joint_states[0]], is_prealign=True)
    if pre_future is None:
        print("ERROR: Could not connect to /stretch_controller")
        node.destroy_node(); rclpy.shutdown(); return
    rclpy.spin_until_future_complete(node, pre_future)
    result_future = pre_future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    node.get_logger().info('Aligned. Stabilizing...')
    time.sleep(1.0)

    # --- TRAJECTORY (arm via action, base via cmd_vel) ---
    print(f"\nReady to execute at {args.speed}x speed.")
    input("Press [ENTER] to PLAY...")

    exec_future = node.send_arm_trajectory(joint_names, joint_states, fps=fps)
    if exec_future is None:
        print("ERROR: Could not connect to /stretch_controller")
        node.destroy_node(); rclpy.shutdown(); return

    # Drive base simultaneously via cmd_vel
    dt = 1.0 / fps
    for i in range(len(joint_states)):
        twist = Twist()
        if i < len(joint_states) - 1:
            twist.linear.x = float((joint_states[i+1, base_trans_idx] - joint_states[i, base_trans_idx]) * fps)
            twist.angular.z = float((joint_states[i+1, base_rot_idx] - joint_states[i, base_rot_idx]) * fps)
        node._cmd_vel_pub.publish(twist)
        time.sleep(dt)

    # Stop base
    node._cmd_vel_pub.publish(Twist())

    # Wait for arm trajectory to finish
    rclpy.spin_until_future_complete(node, exec_future)
    res_future = exec_future.result().get_result_async()
    rclpy.spin_until_future_complete(node, res_future)

    if res_future.result().status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info('SUCCESS')
    else:
        node.get_logger().error('ABORTED: Check stretch_driver terminal.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()