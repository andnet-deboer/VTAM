#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import stretch_body.robot as rb
import time
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from vtam_core.umi_gripper_node import GripperController

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

        # 3. State Tracking for "Locked" Pose
        self.locked_pose = None
        self.last_warning_time = 0

        self.gripper_controller = GripperController(self, self.robot)
        
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self.control_tick)
        
        self.get_logger().info('VTAM Control Loop ready')

    def control_tick(self):
        self.robot.pull_status()
        
        # Attempt to capture the UMI handle pose from the detector
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                'umi_disconnect', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            rotation_quat = quaternion_from_euler(0.424, -0.556, -0.513)
            current_quat = (
                t.transform.rotation.x, 
                t.transform.rotation.y, 
                t.transform.rotation.z, 
                t.transform.rotation.w
            )
            new_quat = quaternion_multiply(current_quat, rotation_quat)
            # Apply 180-degree rotation around z-axis and 180-degree roll
            z_180_quat = quaternion_from_euler(0, 0, 3.14159)
            new_quat = quaternion_multiply(new_quat, z_180_quat)
            roll_180_quat = quaternion_from_euler(0, 3.14159, 0)
            new_quat = quaternion_multiply(new_quat, roll_180_quat)
            t.transform.rotation.x = new_quat[0]
            t.transform.rotation.y = new_quat[1]
            t.transform.rotation.z = new_quat[2]
            t.transform.rotation.w = new_quat[3]
            self.locked_pose = t
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Only warn occasionally to avoid log spam
            current_time = time.time()
            if current_time - self.last_warning_time > 5.0:
                self.get_logger().debug(f'Waiting for UMI detection: {e}')
                self.last_warning_time = current_time

        # Broadcast the dynamic gripper body
        self.broadcast_virtual_gripper()
        self.gripper_controller.tick()
        
        self.robot.push_command()
        self.publish_js()

    def broadcast_virtual_gripper(self):
        """
        Broadcasts link_gripper_s3_body. 
        If UMI is found, it stays at the cube handle.
        If never found, it defaults to the physical wrist.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = 'link_gripper_s3_body'

        if self.locked_pose is not None:
            # Use the locked UMI pose
            t.header.frame_id = 'base_link'
            t.transform = self.locked_pose.transform
        else:
            # Fallback: Attach to the physical wrist roll until first detection
            t.header.frame_id = 'link_wrist_roll'
            t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

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