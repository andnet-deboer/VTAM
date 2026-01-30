#!/usr/bin/env python3
"""
UMI Gripper Controller Node - Uses Stretch Body API
Adaptive baseline handles sticky/hysteresis sensor behavior
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import stretch_body.robot


class UMIGripperNode(Node):
    def __init__(self):
        super().__init__('umi_gripper_controller')

        # --- Parameters ---
        # Adaptive baseline parameters
        self.declare_parameter('initial_baseline', 40.0)
        self.declare_parameter('baseline_decay', 0.01)      # How fast baseline adapts when released
        self.declare_parameter('activation_threshold', 30.0) # Delta above baseline to register touch
        
        # Mapping parameters (based on your observations)
        self.declare_parameter('touch_min', 70.0)   # Light touch magnitude
        self.declare_parameter('touch_max', 500.0)  # Hard press (no need to go to 1100)
        
        # Smoothing
        self.declare_parameter('smoothing', 0.3)
        
        # Stretch gripper range
        self.declare_parameter('gripper_open', 50.0)
        self.declare_parameter('gripper_closed', -20.0)

        # Load parameters
        self.adaptive_baseline = self.get_parameter('initial_baseline').value
        self.baseline_decay = self.get_parameter('baseline_decay').value
        self.activation_threshold = self.get_parameter('activation_threshold').value
        self.touch_min = self.get_parameter('touch_min').value
        self.touch_max = self.get_parameter('touch_max').value
        self.smoothing = self.get_parameter('smoothing').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value

        # --- State ---
        self.smoothed_value = self.adaptive_baseline
        self.is_touching = False
        self.has_data = False
        self.gripper_position = self.gripper_open  # Start open
        self.normalized_grip = 0.0

        # --- Stretch Robot ---
        self.get_logger().info('Connecting to Stretch...')
        self.robot = stretch_body.robot.Robot()
        if not self.robot.startup():
            self.get_logger().error('Failed to start Stretch robot')
            raise RuntimeError('Stretch startup failed')
        
        self.gripper = self.robot.end_of_arm.get_joint('stretch_gripper')
        self.get_logger().info('Stretch connected')

        # --- Publishers ---
        self.pub_width = self.create_publisher(Float32, '/gripper_width', 10)
        self.pub_width_normalized = self.create_publisher(Float32, '/gripper_width_normalized', 10)
        self.pub_baseline = self.create_publisher(Float32, '/debug/adaptive_baseline', 10)
        self.pub_delta = self.create_publisher(Float32, '/debug/delta', 10)

        # --- Subscriber ---
        self.sub_tactile = self.create_subscription(
            Float32MultiArray,
            '/tactile_gripper_controller',
            self.tactile_callback,
            10
        )

        # --- Timer for robot push_command ---
        self.timer = self.create_timer(0.05, self.update_robot)  # 20Hz
        
        # --- Debug logging timer ---
        self.debug_timer = self.create_timer(0.5, self.log_debug)

        self.get_logger().info('UMI Gripper Controller ready (adaptive baseline)')

    def tactile_to_magnitude(self, tactile_data: np.ndarray) -> float:
        """Convert 15-element tactile array to single magnitude value."""
        data = tactile_data.reshape(5, 3)
        magnitudes = np.linalg.norm(data, axis=1)
        return np.mean(magnitudes)

    def tactile_callback(self, msg: Float32MultiArray):
        """Process tactile data with adaptive baseline."""
        self.has_data = True
        
        tactile_data = np.array(msg.data)
        raw_value = self.tactile_to_magnitude(tactile_data)
        
        # Smooth the raw input
        self.smoothed_value = (
            self.smoothing * raw_value + 
            (1 - self.smoothing) * self.smoothed_value
        )
        
        # Compute delta from adaptive baseline
        delta = self.smoothed_value - self.adaptive_baseline
        
        # Publish debug info
        self.publish_float(self.pub_baseline, self.adaptive_baseline)
        self.publish_float(self.pub_delta, delta)
        
        if delta > self.activation_threshold:
            # User is pressing
            self.is_touching = True
            
            # Map: activation_threshold → open, touch_max → closed
            effective_value = delta - self.activation_threshold
            effective_range = self.touch_max - self.touch_min
            self.normalized_grip = np.clip(effective_value / effective_range, 0.0, 1.0)
            
            # Update gripper target
            self.gripper_position = (
                self.gripper_open + 
                self.normalized_grip * (self.gripper_closed - self.gripper_open)
            )
        else:
            # User released - slowly drift baseline toward current value
            self.is_touching = False
            self.adaptive_baseline = (
                self.baseline_decay * self.smoothed_value + 
                (1 - self.baseline_decay) * self.adaptive_baseline
            )
            # Gripper stays where it is - don't snap open
            # (or uncomment below to slowly open when released)
            # self.gripper_position = min(self.gripper_position + 1.0, self.gripper_open)

        # Publish normalized grip for recording
        self.publish_float(self.pub_width_normalized, self.normalized_grip)

    def update_robot(self):
        """Push commands to robot and read state."""
        if self.has_data:
            self.gripper.move_to(self.gripper_position)
        
        self.robot.push_command()
        
        # Publish actual gripper position
        actual_pos = self.gripper.status['pos']
        self.publish_float(self.pub_width, actual_pos)

    def log_debug(self):
        """Print debug info periodically."""
        delta = self.smoothed_value - self.adaptive_baseline
        self.get_logger().info(
            f'smooth={self.smoothed_value:.0f} baseline={self.adaptive_baseline:.0f} '
            f'delta={delta:.0f} touching={self.is_touching} '
            f'grip_norm={self.normalized_grip:.2f} grip_pos={self.gripper_position:.1f}'
        )

    def publish_float(self, pub, value):
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down Stretch...')
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