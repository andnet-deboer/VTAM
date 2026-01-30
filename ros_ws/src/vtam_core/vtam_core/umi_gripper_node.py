#!/usr/bin/env python3
"""
UMI Gripper Controller Node - Simple proportional control with auto-calibration
No hardcoded values - learns from sensor data
"""

import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import stretch_body.robot


class UMIGripperNode(Node):
    def __init__(self):
        super().__init__('umi_gripper_controller')

        # --- Parameters ---
        self.declare_parameter('calibration_seconds', 3.0)
        self.declare_parameter('threshold_percent', 0.04)
        self.declare_parameter('smoothing', 0.3)
        self.declare_parameter('curve_exponent', 0.3)  # <1 = easier to reach full close

        self.calibration_seconds = self.get_parameter('calibration_seconds').value
        self.threshold_percent = self.get_parameter('threshold_percent').value
        self.smoothing = self.get_parameter('smoothing').value
        self.curve_exponent = self.get_parameter('curve_exponent').value

        # --- State ---
        self.baseline = None
        self.max_seen = None
        self.smoothed_value = None
        self.calibration_buffer = []
        self.calibration_start_time = None
        self.ready = False

        # --- Stretch Robot ---
        self.get_logger().info('Connecting to Stretch...')
        self.robot = stretch_body.robot.Robot()
        if not self.robot.startup():
            self.get_logger().error('Failed to start Stretch robot')
            raise RuntimeError('Stretch startup failed')

        self.gripper = self.robot.end_of_arm.get_joint('stretch_gripper')
        
        self.gripper_open = self.gripper.pct_max_open
        self.gripper_closed = -50.0
        
        self.get_logger().info(f'Gripper range: closed={self.gripper_closed}, open={self.gripper_open:.1f}')
        
        self.gripper_position = self.gripper_open

        # --- Open gripper and wait ---
        self.get_logger().info('Opening gripper fully... please wait')
        self.gripper.move_to(self.gripper_open)
        self.robot.push_command()
        
        for i in range(30):
            self.robot.push_command()
            time.sleep(0.1)
        
        self.get_logger().info(f'Gripper at position: {self.gripper.status["pos"]:.2f}')
        self.get_logger().info(f'DO NOT TOUCH SENSOR - calibrating for {self.calibration_seconds}s...')

        # --- Publishers ---
        self.pub_width = self.create_publisher(Float32, '/gripper_width', 10)
        self.pub_normalized = self.create_publisher(Float32, '/gripper_width_normalized', 10)

        # --- Subscriber ---
        self.sub_tactile = self.create_subscription(
            Float32MultiArray,
            '/tactile_gripper_controller',
            self.tactile_callback,
            10
        )

        # --- Timer ---
        self.timer = self.create_timer(0.05, self.update_robot)
        self.debug_timer = self.create_timer(0.5, self.log_debug)

    def tactile_callback(self, msg: Float32MultiArray):
        """Process tactile data."""
        data = np.array(msg.data).reshape(5, 3)
        magnitudes = np.linalg.norm(data, axis=1)
        raw_value = np.mean(magnitudes)

        # --- Calibration phase ---
        if self.baseline is None:
            now = self.get_clock().now()
            if self.calibration_start_time is None:
                self.calibration_start_time = now

            elapsed = (now - self.calibration_start_time).nanoseconds / 1e9
            self.calibration_buffer.append(raw_value)

            if elapsed >= self.calibration_seconds:
                self.baseline = np.mean(self.calibration_buffer)
                self.smoothed_value = self.baseline
                # Start max_seen at 2x baseline - will adapt up from there
                self.max_seen = self.baseline * 1.50
                self.ready = True
                self.get_logger().info(f'Calibrated! Baseline={self.baseline:.0f}')
                self.get_logger().info('Ready! Squeeze to close gripper.')
            return

        # --- Normal operation ---
        self.smoothed_value = (
            self.smoothing * raw_value +
            (1 - self.smoothing) * self.smoothed_value
        )

        # Update max seen - adapt UP quickly, DOWN slowly
        if self.smoothed_value > self.max_seen:
            self.max_seen = self.smoothed_value  # Instant adapt up
        else:
            self.max_seen = 0.9995 * self.max_seen + 0.0005 * self.smoothed_value  # Very slow decay

        # Calculate threshold and range
        threshold = self.baseline * (1 + self.threshold_percent)
        range_size = self.max_seen - threshold

        # Map to normalized grip (0 = no squeeze, 1 = max squeeze)
        if self.smoothed_value < threshold or range_size <= 0:
            normalized = 0.0
        else:
            normalized = (self.smoothed_value - threshold) / range_size
            normalized = np.clip(normalized, 0.0, 1.0)

        # Apply curve: exponent < 1 makes it easier to reach full close
        # 0.5 = square root curve (50% force = 71% closed)
        curved = np.power(normalized, self.curve_exponent)

        # Map to gripper position
        self.gripper_position = self.gripper_open - curved * (self.gripper_open - self.gripper_closed)

        # Publish
        msg_out = Float32()
        msg_out.data = float(curved)
        self.pub_normalized.publish(msg_out)

    def update_robot(self):
        """Send commands to robot."""
        if self.ready:
            self.gripper.move_to(self.gripper_position)
        else:
            self.gripper.move_to(self.gripper_open)
        
        self.robot.push_command()

        msg = Float32()
        msg.data = float(self.gripper.status['pos'])
        self.pub_width.publish(msg)

    def log_debug(self):
        """Print status."""
        if self.baseline is None:
            elapsed = 0.0
            if self.calibration_start_time is not None:
                elapsed = (self.get_clock().now() - self.calibration_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Calibrating... {elapsed:.1f}/{self.calibration_seconds}s')
            return

        threshold = self.baseline * (1 + self.threshold_percent)
        delta = self.smoothed_value - self.baseline
        delta_percent = (delta / self.baseline) * 100 if self.baseline > 0 else 0

        range_size = self.max_seen - threshold
        normalized = 0.0
        if self.smoothed_value >= threshold and range_size > 0:
            normalized = (self.smoothed_value - threshold) / range_size
            normalized = np.clip(normalized, 0.0, 1.0)
        
        curved = np.power(normalized, self.curve_exponent)

        self.get_logger().info(
            f'delta={delta_percent:.1f}% raw_norm={normalized:.2f} curved={curved:.2f} pos={self.gripper_position:.1f}'
        )

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        self.robot.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UMIGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()