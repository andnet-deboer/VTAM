#!/usr/bin/env python3
"""
Gripper Controller
Commands gripper via stretch_driver's action server instead of direct hardware.
"""

import numpy as np
import rclpy
import time
import os
import yaml
import hello_helpers.hello_misc as hm
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult


class GripperController(hm.HelloNode):
    """Tactile-driven gripper control via stretch_driver."""

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'gripper_controller', 'gripper_controller',
                          wait_for_first_pointcloud=False)

        # Parameters
        self.add_on_set_parameters_callback(self._on_params_changed)

        self.declare_parameter('update_calibration', False)
        self.declare_parameter('threshold_percent', 0.001)
        self.declare_parameter('smoothing', 0.9)
        self.declare_parameter('close_effort', 1000.0)
        self.declare_parameter('curve_exponent', 0.9)

        self.curve_exponent = self.get_parameter('curve_exponent').value
        self.close_effort = self.get_parameter('close_effort').value
        self.update_mode = self.get_parameter('update_calibration').value
        self.threshold_percent = self.get_parameter('threshold_percent').value
        self.smoothing = self.get_parameter('smoothing').value

        # State
        self.baseline = None
        self.max_seen = None
        self.smoothed_value = 0.0
        self.ready = False
        self.calib_state = "IDLE"
        self.calib_start_time = None
        self.buffer = []


        self.gripper_open = 1.05
        self.gripper_closed = -0.2
        self.gripper_position = self.gripper_open
        self.current_gripper_pos = self.gripper_open
        self.initialized = False

        # Paths
        self.pkg_share = get_package_share_directory('vtam_core')
        self.config_path = os.path.join(self.pkg_share, 'config', 'umi_trigger_calibration.yaml')

        # Calibration
        if self.update_mode:
            self._start_calibration()
        else:
            self._load_calibration()

        # ROS comms
        self.pub_width = self.create_publisher(Float32, '/gripper_width', 10)
        self.pub_normalized = self.create_publisher(Float32, '/gripper_width_normalized', 10)
        self.create_subscription(Float32MultiArray, '/tactile_gripper_controller', self._tactile_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.create_timer(1.0, self._log_debug)

        self.get_logger().info("GripperController ready (via stretch_driver)")

    # -- Joint state feedback --

    def _js_cb(self, msg):
        try:
            idx = msg.name.index('joint_gripper_finger_left')
            self.current_gripper_pos = msg.position[idx]

            if not self.initialized:
                self.initialized = True
                self.move_to_pose({'joint_gripper_finger_left': self.gripper_open}, blocking=False)
                self.get_logger().info("Gripper commanded to open position")

            out = Float32()
            out.data = float(self.current_gripper_pos)
            self.pub_width.publish(out)
        except ValueError:
            pass

    # -- Parameter tuning --

    def _on_params_changed(self, params):
        for param in params:
            if param.name == 'smoothing':
                self.smoothing = param.value
            elif param.name == 'threshold_percent':
                self.threshold_percent = param.value
            elif param.name == 'close_effort':
                self.close_effort = param.value
            elif param.name == 'curve_exponent':
                self.curve_exponent = param.value
        self.get_logger().info("Live-tuned parameters updated.")
        return SetParametersResult(successful=True)

    # -- Tactile callback --

    def _tactile_cb(self, msg: Float32MultiArray):
        if self.gripper_open is None:
            return 
        data = np.array(msg.data).reshape(5, 3)
        raw_val = np.mean(np.linalg.norm(data, axis=1))

        if self.update_mode and not self.ready:
            self._run_calibration(raw_val)
            return
        if not self.ready:
            return

        self.smoothed_value = self.smoothing * raw_val + (1 - self.smoothing) * self.smoothed_value

        threshold = self.baseline * (1 + self.threshold_percent)

        if self.smoothed_value <= threshold:
            normalized = 0.0
        else:
            normalized = np.clip(
                (self.smoothed_value - threshold) / (self.close_effort - threshold),
                0.0, 1.0
            )
            normalized = np.power(normalized, self.curve_exponent)

        if normalized < 0.01:
            target = self.gripper_open
        else:
            total_travel = self.gripper_open - self.gripper_closed
            target = self.gripper_open - (normalized * total_travel)

        # Command gripper through stretch_driver
        if abs(target - self.gripper_position) > 0.01:
            self.gripper_position = target
            self.move_to_pose({'joint_gripper_finger_left': target}, blocking=False)

        out = Float32()
        out.data = float(normalized)
        self.pub_normalized.publish(out)

    # -- Calibration --

    def _start_calibration(self):
        self.calib_state = "PHASE_1_REST"
        self.calib_start_time = time.time()
        self.buffer = []
        self._speak("Starting calibration. Please do not touch the tactile sensor.")

    def _run_calibration(self, raw_val):
        elapsed = time.time() - self.calib_start_time
        if self.calib_state == "PHASE_1_REST":
            self.buffer.append(raw_val)
            if elapsed >= 4.0:
                self.baseline = np.mean(self.buffer)
                self.calib_state = "PHASE_2_SQUEEZE"
                self.calib_start_time = time.time()
                self.buffer = []
                self._speak("Now, squeeze the sensor hard.")
        elif self.calib_state == "PHASE_2_SQUEEZE":
            self.buffer.append(raw_val)
            if elapsed >= 4.0:
                self.max_seen = np.percentile(self.buffer, 90)
                self.calib_state = "PHASE_3_VERIFY"
                self.calib_start_time = time.time()
                self.buffer = []
                self._speak("Release the sensor.")
        elif self.calib_state == "PHASE_3_VERIFY":
            self.buffer.append(raw_val)
            if elapsed >= 4.0:
                self._save_calibration()
                self.ready = True
                self._speak("Calibration successful.")

    def _load_calibration(self):
        try:
            with open(self.config_path, 'r') as f:
                data = yaml.safe_load(f)
                params = data['/**']['ros__parameters']
                self.baseline = params['baseline']
                self.max_seen = params['max_seen']
                self.ready = True
                self.get_logger().info(f"Calibration loaded: baseline={self.baseline:.1f}, max={self.max_seen:.1f}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")

    def _save_calibration(self):
        data = {'/**': {'ros__parameters': {'baseline': float(self.baseline), 'max_seen': float(self.max_seen)}}}
        with open(self.config_path, 'w') as f:
            yaml.dump(data, f)

    def _speak(self, text):
        self.get_logger().info(f"Robot says: {text}")
        os.system(f"espeak-ng '{text}' -s 150 &")

    def _log_debug(self):
        if self.ready:
            self.get_logger().debug(f"Tactile: {self.smoothed_value:.0f} | Gripper: {self.current_gripper_pos:.2f}")


def main():
    node = GripperController()
    node.main()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()