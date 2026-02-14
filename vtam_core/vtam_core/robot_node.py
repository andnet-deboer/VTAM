#!/usr/bin/env python3
"""
VTAM Robot Node — Acts as the primary driver.
Publishes /joint_states so robot_state_publisher can update TF.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import stretch_body.robot as rb
import signal
import time
from tf2_ros import Buffer, TransformListener

# Import behaviors
from vtam_core.umi_pose_tracker import HeadTrackerNode
from vtam_core.umi_gripper_node import GripperController

class RobotNode(Node):
    def __init__(self):
        super().__init__('vtam_robot_node')

        # ─── 1. Hardware Interface ───
        self.get_logger().info("Starting Stretch hardware...")
        self.robot = rb.Robot()
        if not self.robot.startup():
            self.get_logger().error("Robot startup failed!")
            raise RuntimeError("Robot startup failed")
        self.get_logger().info("Stretch hardware OK")

        # ─── 2. TF & Joint State Setup ───
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # CRITICAL: We must publish joint states for TF to work
        self.js_pub = self.create_publisher(JointState, '/joint_states', 60)

        # ─── 3. Attach Behaviors ───
        self.head_tracker = HeadTrackerNode(self, self.robot)
        self.gripper = GripperController(self, self.robot)

        # ─── 4. Loops ───
        self.create_timer(0.02, self._tick)  # 50Hz Control Loop

        # ─── 5. Cleanup ───
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        self._shutting_down = False

        self.get_logger().info("VTAM Robot Node ready (Driver Mode)")

    def _tick(self):
        if self._shutting_down: return

        # 1. Read Hardware
        self.robot.pull_status()

        # 2. Update Behaviors
        self.head_tracker.tick()
        self.gripper.tick()

        # 3. Publish State (So TF works)
        self._publish_joint_states()

        # 4. Write Hardware
        self.robot.push_command()

    def _publish_joint_states(self):
        """
        Maps stretch_body status to ROS JointState.
        Essential for robot_state_publisher to link base_link -> camera.
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        
        # Helper to safely get position
        def get_pos(joint_name, status_dict):
            try:
                return float(status_dict[joint_name]['pos'])
            except (KeyError, TypeError):
                return 0.0

        # --- HEAD ---
        # Note: stretch_body puts these in robot.head.status
        if 'head' in self.robot.status:
            js.name.extend(['joint_head_pan', 'joint_head_tilt'])
            js.position.extend([
                get_pos('head_pan', self.robot.status['head']),
                get_pos('head_tilt', self.robot.status['head'])
            ])

        # --- LIFT ---
        if 'lift' in self.robot.status:
            js.name.append('joint_lift')
            js.position.append(get_pos('pos', self.robot.status['lift']))

        # --- ARM (Telescoping Split) ---
        # The URDF has 4 links (l0..l3). We divide total extension by 4.
        if 'arm' in self.robot.status:
            arm_total = self.robot.status['arm']['pos']
            segment_pos = arm_total / 4.0
            for i in range(4):
                js.name.append(f'joint_arm_l{i}')
                js.position.append(segment_pos)

        # --- WRIST & GRIPPER ---
        if 'end_of_arm' in self.robot.status:
            eoa = self.robot.status['end_of_arm']
            
            # Yaw
            js.name.append('joint_wrist_yaw')
            js.position.append(get_pos('wrist_yaw', eoa))
            
            # Pitch/Roll (if DexWrist)
            if 'wrist_pitch' in eoa:
                js.name.append('joint_wrist_pitch')
                js.position.append(get_pos('wrist_pitch', eoa))
            if 'wrist_roll' in eoa:
                js.name.append('joint_wrist_roll')
                js.position.append(get_pos('wrist_roll', eoa))

            # Gripper (Map to both fingers)
            # stretch_body usually gives raw radians in 'pos' for gripper
            grip_pos = get_pos('stretch_gripper', eoa)
            js.name.extend(['joint_gripper_finger_left', 'joint_gripper_finger_right'])
            js.position.extend([grip_pos, grip_pos])

        self.js_pub.publish(js)

    def _signal_handler(self, sig, frame):
        self.stop()
        raise KeyboardInterrupt

    def stop(self):
        self._shutting_down = True
        time.sleep(0.1)
        try: self.robot.stop()
        except: pass

def main():
    rclpy.init()
    node = RobotNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()