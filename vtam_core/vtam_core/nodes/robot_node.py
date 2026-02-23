#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import stretch_body.robot as rb
import time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from vtam_core.nodes.umi_gripper_node import GripperController
from vtam_core.umi_pose_tracker import HeadPoseTracker

# Add these imports
try:
    from tf_transformations import quaternion_multiply, quaternion_from_euler
except ImportError:
    from scipy.spatial.transform import Rotation as R
    def quaternion_from_euler(roll, pitch, yaw):
        r = R.from_euler('xyz', [roll, pitch, yaw])
        return r.as_quat()
    def quaternion_multiply(q1, q2):
        r1 = R.from_quat(q1)
        r2 = R.from_quat(q2)
        return (r1 * r2).as_quat()

class VtamControlLoop(Node):
    def __init__(self):
        super().__init__('vtam_control_loop')
        
        self.get_logger().info('Initializing VTAM Control Loop...')
        
        # 1. Hardware
        try:
            self.robot = rb.Robot()
            self.robot.startup()
            self.get_logger().info('Stretch robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            raise

        # 2. TF Infrastructure
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_warning_time = 0

        self.gripper_controller = GripperController(self, self.robot)
        
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self.control_tick)

        self.sync_pub = self.create_publisher(JointState, '/sync_pulse', 10)

        self.create_timer(0.1, self.publish_sync_pulse)

        self.head_tracker = HeadPoseTracker(self, self.robot)
        
        self.get_logger().info('VTAM Control Loop ready')

    def control_tick(self):
        self.robot.pull_status()
        self.gripper_controller.tick()
        self.head_tracker.tick()
        
        self.robot.push_command()
        self.publish_js()

    def publish_sync_pulse(self):
        """The heartbeat that tells cameras and eFlesh to 'snap' their data"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "master_clock"
        # Optional: Include the latest robot poses in this heartbeat to save bandwidth
        self.sync_pub.publish(msg)

    def publish_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'joint_head_pan', 'joint_head_tilt', 'joint_lift', 
            'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
            'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
            'joint_gripper_finger_left', 'joint_gripper_finger_right'
        ]
        
        h = self.robot.head.status
        a = self.robot.arm.status
        l = self.robot.lift.status
        w = self.robot.end_of_arm.status
        
        arm_p = a['pos']
        msg.position = [
            float(h['head_pan']['pos']), float(h['head_tilt']['pos']), float(l['pos']),
            float(arm_p/4), float(arm_p/4), float(arm_p/4), float(arm_p/4),
            float(w['wrist_yaw']['pos']), float(w['wrist_pitch']['pos']), 
            float(w['wrist_roll']['pos']),
            float(w['stretch_gripper']['pos']), float(w['stretch_gripper']['pos'])
        ]
        self.js_pub.publish(msg)
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down robot...')
        try:
            self.robot.stop()
        except Exception as e:
            self.get_logger().error(f'Error during robot shutdown: {e}')

def main():
    rclpy.init()
    node = VtamControlLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()