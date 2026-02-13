#!/usr/bin/env python3
import numpy as np
import time
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState # NEW
from ament_index_python.packages import get_package_share_directory
import stretch_body.robot

class UMIGripperNode(Node):
    def __init__(self):
        super().__init__('umi_gripper_controller')

        # --- Parameters ---
        self.declare_parameter('update_calibration', False)
        self.declare_parameter('threshold_percent', 0.10)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('curve_exponent', 0.4)

        self.update_mode = self.get_parameter('update_calibration').value
        self.threshold_percent = self.get_parameter('threshold_percent').value
        self.smoothing = self.get_parameter('smoothing').value
        self.curve_exponent = self.get_parameter('curve_exponent').value

        # --- State Variables ---
        self.baseline = None
        self.max_seen = None
        self.smoothed_value = 0.0
        self.ready = False
        self.calib_state = "IDLE" 
        self.calib_start_time = None
        self.buffer = []

        # --- Paths ---
        self.pkg_share = get_package_share_directory('vtam_core')
        self.config_path = os.path.join(self.pkg_share, 'config', 'umi_trigger_calibration.yaml')

        # --- Setup Stretch Hardware ---
        self.get_logger().info('Connecting to Stretch hardware...')
        self.robot = stretch_body.robot.Robot()
        if not self.robot.startup():
            self.get_logger().error('Stretch startup failed!')
            raise RuntimeError('Stretch startup failed')

        self.gripper = self.robot.end_of_arm.get_joint('stretch_gripper')
        self.gripper_open = self.gripper.pct_max_open
        self.gripper_closed = -50.0
        self.gripper_position = self.gripper_open

        if self.update_mode:
            self.start_calibration_sequence()
        else:
            self.load_calibration()

        # --- ROS Comms ---
        self.pub_width = self.create_publisher(Float32, '/gripper_width', 10)
        self.pub_normalized = self.create_publisher(Float32, '/gripper_width_normalized', 10)
        self.js_pub = self.create_publisher(JointState, '/joint_states', 10) # NEW
        
        self.sub_tactile = self.create_subscription(
            Float32MultiArray, '/tactile_gripper_controller', self.tactile_callback, 10
        )

        self.timer = self.create_timer(0.05, self.update_robot)
        self.debug_timer = self.create_timer(1.0, self.log_debug)

    def speak(self, text):
        self.get_logger().info(f'Robot says: {text}')
        os.system(f"espeak-ng '{text}' -s 150 &")

    def start_calibration_sequence(self):
        self.calib_state = "PHASE_1_REST"
        self.calib_start_time = time.time()
        self.buffer = []
        self.speak("Starting calibration. Please do not touch the tactile sensor.")

    def load_calibration(self):
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
                params = data['/**']['ros__parameters']
                self.baseline = params['baseline']
                self.max_seen = params['max_seen']
                self.ready = True
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')

    def save_calibration(self):
        data = {'/**': {'ros__parameters': {'baseline': float(self.baseline), 'max_seen': float(self.max_seen)}}}
        with open(self.config_path, 'w') as f:
            yaml.dump(data, f)

    def tactile_callback(self, msg: Float32MultiArray):
        data = np.array(msg.data).reshape(5, 3)
        raw_val = np.mean(np.linalg.norm(data, axis=1))

        if self.update_mode and not self.ready:
            elapsed = time.time() - self.calib_start_time
            if self.calib_state == "PHASE_1_REST":
                self.buffer.append(raw_val)
                if elapsed >= 4.0:
                    self.baseline = np.mean(self.buffer)
                    self.calib_state = "PHASE_2_SQUEEZE"
                    self.calib_start_time = time.time()
                    self.buffer = []
                    self.speak("Now, squeeze the sensor hard.")
            elif self.calib_state == "PHASE_2_SQUEEZE":
                self.buffer.append(raw_val)
                if elapsed >= 4.0:
                    self.max_seen = np.percentile(self.buffer, 90)
                    self.calib_state = "PHASE_3_VERIFY"
                    self.calib_start_time = time.time()
                    self.buffer = []
                    self.speak("Release the sensor.")
            elif self.calib_state == "PHASE_3_VERIFY":
                self.buffer.append(raw_val)
                if elapsed >= 4.0:
                    self.save_calibration()
                    self.ready = True
                    self.speak("Calibration successful.")
            return

        if not self.ready: return
        self.smoothed_value = (self.smoothing * raw_val + (1 - self.smoothing) * self.smoothed_value)
        threshold = self.baseline * (1 + self.threshold_percent)
        range_size = self.max_seen - threshold
        normalized = np.clip((self.smoothed_value - threshold) / range_size, 0.0, 1.0) if range_size > 0 else 0.0
        curved = np.power(normalized, self.curve_exponent)
        self.gripper_position = self.gripper_open if curved < 0.01 else self.gripper_open - curved * (self.gripper_open - self.gripper_closed)

        msg_out = Float32(); msg_out.data = float(curved); self.pub_normalized.publish(msg_out)

    def update_robot(self):
        """DIRECT hardware control for zero latency + JointState publishing for TFs."""
        # 1. Pull status for all joints so we can update the TF tree
        self.robot.pull_status() 
        
        # 2. Direct Low-Latency Hardware Command
        if self.ready:
            self.gripper.move_to(self.gripper_position)
        else:
            self.gripper.move_to(self.gripper_open)
        self.robot.push_command()
        
        # 3. Publish FULL JointState message to 'unlock' RViz and TFs
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        
        # Pull names and positions for everything (lift, arm, head, etc.)
        # This ensures your camera TFs are calculated correctly by robot_state_publisher
        js.name = []
        js.position = []
        for joint_name in self.robot.status:
            if 'pos' in self.robot.status[joint_name]:
                js.name.append(joint_name)
                js.position.append(float(self.robot.status[joint_name]['pos']))
        
        self.js_pub.publish(js)

        # Standard width publishing
        msg = Float32()
        msg.data = float(self.gripper.status['pos'])
        self.pub_width.publish(msg)

    def log_debug(self):
        if self.ready:
            self.get_logger().info(f'Tactile: {self.smoothed_value:.0f} | Pos: {self.gripper_position:.2f}')

    def destroy_node(self):
        self.robot.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UMIGripperNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()