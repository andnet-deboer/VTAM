#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
import stretch_body.robot as rb
import time
from vtam_core.nodes.umi_gripper_node import GripperController
from vtam_core.umi_pose_tracker import HeadPoseTracker

class VtamControlLoop(Node):
    def __init__(self):
        super().__init__('vtam_control_loop')
        self.get_logger().info('Initializing Multi-Threaded 50Hz/30Hz VTAM Loop...')
        
        # 1. Hardware Connection
        try:
            self.robot = rb.Robot()
            if not self.robot.startup():
                self.get_logger().error('Hardware locked! Run "stretch_free_robot_process.py"')
                raise RuntimeError("Hardware Lock")
            self.get_logger().info('Stretch robot connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise

        # 2. Infrastructure
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sync_pub = self.create_publisher(JointState, '/sync_pulse', 10)

        # 3. Controllers & Tracker
        self.gripper_controller = GripperController(self, self.robot)
        self.head_tracker = HeadPoseTracker(self, self.robot)

        # 4. State Management
        self.tick_count = 0
        self.lock = threading.Lock() # Prevents thread collision on the robot object
        
        # 5. ASYNC TRACKER THREAD
        # Moving the tracker here prevents its 60ms spikes from killing the 30Hz sync
        self.tracker_thread = threading.Thread(target=self._tracker_loop, daemon=True)
        self.tracker_thread.start()

        # 6. MASTER CONTROL TIMER (50Hz / 0.02s)
        self.create_timer(0.02, self.control_tick)
        
        self.get_logger().info('VTAM System Ready: Tracking decoupled from Synchronization.')

    def _tracker_loop(self):
        """Dedicated background thread for EE pose tracking."""
        # This loop runs as fast as the CPU allows without blocking the heartbeat
        self.get_logger().info('Async Tracker Thread Started')
        while rclpy.ok():
            try:
                with self.lock:
                    # Tracker is essential for EE poses, so we run it constantly
                    self.head_tracker.tick()
                # Tiny sleep to prevent 100% CPU burn-up
                time.sleep(0.033)
            except Exception as e:
                self.get_logger().error(f"Tracker thread error: {e}")

    def control_tick(self):
        """High-priority heartbeat for hardware and data logging."""
        start_time = self.get_clock().now()
        
        try:
            with self.lock:
                # 1. Hardware Reflexes (50Hz)
                self.robot.pull_status()
                self.gripper_controller.tick()
                self.robot.push_command()
                self.publish_js()
            self._publish_head_js()

            # 2. Data Sync Pulse (~30Hz)
            # We pulse 3 times every 5 ticks (3/5 * 50 = 30Hz)
            self.tick_count += 1
            if self.tick_count % 5 in [0, 2, 4]:
                self.publish_sync_pulse()
            
            if self.tick_count >= 5:
                self.tick_count = 0

            # 3. Performance Monitor
            duration = (self.get_clock().now() - start_time).nanoseconds / 1e6
            if duration > 20.0:
                self.get_logger().debug(f"Jitter detected: {duration:.2f}ms")

        except Exception as e:
            self.get_logger().error(f"Control loop failure: {e}")

    def publish_sync_pulse(self):
        """The heartbeat that tells cameras to 'snap' their data."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "master_clock"
        self.sync_pub.publish(msg)

    def publish_js(self):
        """Publishes JointStates for RViz and TF tree."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'joint_head_pan', 'joint_head_tilt', 'joint_lift', 
            'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
            'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
            'joint_gripper_finger_left', 'joint_gripper_finger_right'
        ]
        
        h, a, l, w = self.robot.head.status, self.robot.arm.status, self.robot.lift.status, self.robot.end_of_arm.status
        arm_p = a.get('pos', 0.0)
        
        try:
            msg.position = [
                float(h.get('head_pan', {}).get('pos', 0.0)),
                float(h.get('head_tilt', {}).get('pos', 0.0)),
                float(l.get('pos', 0.0)),
                float(arm_p/4), float(arm_p/4), float(arm_p/4), float(arm_p/4),
                float(w.get('wrist_yaw', {}).get('pos', 0.0)),
                float(w.get('wrist_pitch', {}).get('pos', 0.0)), 
                float(w.get('wrist_roll', {}).get('pos', 0.0)),
                float(w.get('stretch_gripper', {}).get('pos', 0.0)),
                float(w.get('stretch_gripper', {}).get('pos', 0.0))
            ]
            self.js_pub.publish(msg)
        except Exception:
            pass

    def shutdown(self):
        self.get_logger().info('Emergency Shutdown...')
        try:
            self.robot.stop()
        except:
            pass
    
    def _publish_head_js(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_head_pan', 'joint_head_tilt']
        try:
            msg.position = [
                float(self.robot.head.status['head_pan']['pos']),
                float(self.robot.head.status['head_tilt']['pos'])
            ]
            self.js_pub.publish(msg)  # reuse existing publisher
        except Exception:
            pass

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