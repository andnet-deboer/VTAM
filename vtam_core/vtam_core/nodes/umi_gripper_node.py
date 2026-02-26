#!/usr/bin/env python3
"""
Gripper Controller behavior — attaches to a RobotNode.
Does NOT own rb.Robot() or call push_command(). The parent node handles that.
"""

from platform import node

import numpy as np
import time
import os
import yaml
from std_msgs.msg import Float32MultiArray, Float32
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult


class GripperController:
    """Tactile-driven gripper control."""

    def __init__(self, node, robot):
        """
        Args:
            node: rclpy.Node — used for params, logging, subscriptions
            robot: stretch_body.robot.Robot — shared hardware handle
        """
        self.node = node
        self.robot = robot
        self.logger = node.get_logger()

        # ─── Parameters ───
        self.node.add_on_set_parameters_callback(self._on_params_changed)

        node.declare_parameter('update_calibration', False)
        node.declare_parameter('threshold_percent', 0.001)
        node.declare_parameter('smoothing', 0.9)
        node.declare_parameter('close_effort', 1000.0)  # sensor magnitude for full closure
        node.declare_parameter('curve_exponent', 0.9)

        self.curve_exponent = node.get_parameter('curve_exponent').value
        self.close_effort = node.get_parameter('close_effort').value
        self.update_mode = node.get_parameter('update_calibration').value
        self.threshold_percent = node.get_parameter('threshold_percent').value
        self.smoothing = node.get_parameter('smoothing').value

        # ─── State ───
        self.baseline = None
        self.max_seen = None
        self.smoothed_value = 0.0
        self.ready = False
        self.calib_state = "IDLE"
        self.calib_start_time = None
        self.buffer = []

        # ─── Paths ───
        self.pkg_share = get_package_share_directory('vtam_core')
        self.config_path = os.path.join(self.pkg_share, 'config', 'umi_trigger_calibration.yaml')

        # ─── Gripper hardware ref ───
        self.gripper = self.robot.end_of_arm.get_joint('stretch_gripper')
        self.gripper_open = self.gripper.pct_max_open
        self.gripper_closed = -50.0
        self.gripper_position = self.gripper_open

        # ─── Calibration ───
        if self.update_mode:
            self._start_calibration()
        else:
            self._load_calibration()

        # ─── ROS comms ───
        self.pub_width = node.create_publisher(Float32, '/gripper_width', 10)
        self.pub_normalized = node.create_publisher(Float32, '/gripper_width_normalized', 10)
        node.create_subscription(Float32MultiArray, '/tactile_gripper_controller', self._tactile_cb, 10)

        # Debug log timer
        node.create_timer(1.0, self._log_debug)

        self.logger.info("GripperController attached")

    def tick(self):
        """Called every control cycle. Moves gripper but does NOT push_command."""
        if self.ready:
            self.gripper.move_to(self.gripper_position)
        else:
            self.gripper.move_to(self.gripper_open)

        # Publish width
        msg = Float32()
        msg.data = float(self.gripper.status['pos'])
        self.pub_width.publish(msg)

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
                
        self.logger.info("Live-tuned parameters updated.")
        return SetParametersResult(successful=True)

    # ─── Tactile callback ───

    def _tactile_cb(self, msg: Float32MultiArray):
        data = np.array(msg.data).reshape(5, 3)
        raw_val = np.mean(np.linalg.norm(data, axis=1))

        if self.update_mode and not self.ready:
            self._run_calibration(raw_val)
            return
        if not self.ready:
            return

        self.smoothed_value = self.smoothing * raw_val + (1 - self.smoothing) * self.smoothed_value

        # Dead zone: ignore anything below this
        threshold = self.baseline * (1 + self.threshold_percent)

        # Linear map from threshold → close_effort into 0.0 → 1.0
        # close_effort = how hard you need to squeeze to fully close
        if self.smoothed_value <= threshold:
            normalized = 0.0
        else:
            normalized = np.clip(
                (self.smoothed_value - threshold) / (self.close_effort - threshold),
                0.0, 1.0
            )
            normalized = np.power(normalized, self.curve_exponent)

        if normalized < 0.01:
            self.gripper_position = self.gripper_open
        else:
            total_travel = self.gripper_open - self.gripper_closed
            self.gripper_position = self.gripper_open - (normalized * total_travel)

        out = Float32()
        out.data = float(normalized)
        self.pub_normalized.publish(out)
        
    # ─── Calibration ───

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
                self.logger.info(f"Calibration loaded: baseline={self.baseline:.1f}, max={self.max_seen:.1f}")
        except Exception as e:
            self.logger.error(f"Failed to load calibration: {e}")

    def _save_calibration(self):
        data = {'/**': {'ros__parameters': {'baseline': float(self.baseline), 'max_seen': float(self.max_seen)}}}
        with open(self.config_path, 'w') as f:
            yaml.dump(data, f)

    def _speak(self, text):
        self.logger.info(f"Robot says: {text}")
        os.system(f"espeak-ng '{text}' -s 150 &")

    def _log_debug(self):
        if self.ready:
            self.logger.debug(f"Tactile: {self.smoothed_value:.0f} | Gripper: {self.gripper_position:.2f}")