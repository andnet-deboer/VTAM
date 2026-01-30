#!/usr/bin/env python3
import os
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32MultiArray, Float32
import stretch_body.robot

class UMINode(Node):
    """
    Integrated controller for Stretch 3 featuring high-frequency head 
    visual servoing and tactile-based proportional gripper control.
    """

    def __init__(self):
        super().__init__('umi_node')

        # --- Hardware Interface ---
        self.robot = stretch_body.robot.Robot()
        if not self.robot.startup():
            self.get_logger().error("Hardware communication failed. Check Run-Stop.")
            os._exit(1)

        # --- Configuration & Safety Limits ---
        self.declare_parameter('threshold_percent', 0.04)
        self.threshold = self.get_parameter('threshold_percent').value
        
        self.pan_range = [-3.8, 1.5]
        self.tilt_range = [-1.5, 0.3]
        self.kp, self.kd = 3.5, 0.15
        self.deadzone = 0.005
        self.max_velocity = 2.0

        # --- Joint Initialization ---
        self.gripper = self.robot.end_of_arm.get_joint('stretch_gripper')
        self.gripper_open = self.gripper.pct_max_open
        self.gripper_closed = -50.0

        # --- State Management ---
        self.head_error = np.zeros(2)  # [pan, tilt]
        self.prev_error = np.zeros(2)
        self.last_pose_time = 0.0
        self.state_lock = threading.Lock()

        self.baseline = None
        self.max_force_seen = None
        self.smoothed_tactile = None
        self.calibration_buffer = []
        self.is_calibrated = False

        # --- ROS Communication ---
        self.create_subscription(PoseArray, 'aruco_poses', self._pose_callback, 1)
        self.create_subscription(Float32MultiArray, '/tactile_gripper_controller', self._tactile_callback, 10)
        self.pub_width = self.create_publisher(Float32, '/gripper_width', 10)

        # --- Control Loops ---
        self.is_running = True
        self.head_thread = threading.Thread(target=self._head_control_loop, daemon=True)
        self.head_thread.start()
        self.create_timer(0.05, self._update_gripper)

        self.get_logger().info("UMI_Node initialized and operational.")

    def _pose_callback(self, msg):
        """Asynchronous update of visual error signals."""
        if not msg.poses:
            return
        with self.state_lock:
            # Mapping optical frame to robot kinematics
            self.head_error[0] = msg.poses[0].position.y    # Pan error
            self.head_error[1] = -msg.poses[0].position.x   # Tilt error
            self.last_pose_time = time.time()

    def _tactile_callback(self, msg):
        """Process tactile magnitudes and handle auto-calibration."""
        data = np.array(msg.data).reshape(5, 3)
        current_force = np.mean(np.linalg.norm(data, axis=1))

        if not self.is_calibrated:
            self._handle_calibration(current_force)
            return

        # Exponential moving average for signal stability
        self.smoothed_tactile = 0.3 * current_force + 0.7 * self.smoothed_tactile
        self.max_force_seen = max(self.max_force_seen, self.smoothed_tactile)
        
        self._calculate_gripper_target()

    def _handle_calibration(self, val):
        self.calibration_buffer.append(val)
        if len(self.calibration_buffer) > 60:  # ~1 second at 60Hz
            self.baseline = np.mean(self.calibration_buffer)
            self.smoothed_tactile = self.baseline
            self.max_force_seen = self.baseline * 1.5
            self.is_calibrated = True
            self.get_logger().info(f"Tactile calibrated. Baseline: {self.baseline:.2f}")

    def _calculate_gripper_target(self):
        force_threshold = self.baseline * (1 + self.threshold)
        dynamic_range = self.max_force_seen - force_threshold
        
        normalized = np.clip((self.smoothed_tactile - force_threshold) / dynamic_range 
                             if dynamic_range > 0 else 0, 0, 1)
        
        # Power curve for intuitive force-to-closure mapping
        curved_response = np.power(normalized, 0.3)
        self.target_gripper_pos = self.gripper_open - curved_response * (self.gripper_open - self.gripper_closed)

    def _head_control_loop(self):
        """High-frequency PD velocity control loop (100Hz)."""
        dt = 0.01
        while self.is_running:
            start_time = time.time()
            with self.state_lock:
                err = np.copy(self.head_error)
                data_age = time.time() - self.last_pose_time

            if data_age > 0.5:
                # Watchdog: Stop motion if vision is lost
                self.robot.head.get_joint('head_pan').set_velocity(0)
                self.robot.head.get_joint('head_tilt').set_velocity(0)
            else:
                # Apply deadzone and PD logic
                err[np.abs(err) < self.deadzone] = 0.0
                derivative = (err - self.prev_error) / dt
                velocity = np.clip(self.kp * err + self.kd * derivative, -self.max_velocity, self.max_velocity)
                
                self.robot.head.get_joint('head_pan').set_velocity(velocity[0])
                self.robot.head.get_joint('head_tilt').set_velocity(velocity[1])
                self.prev_error = err

            self.robot.push_command()
            time.sleep(max(0, dt - (time.time() - start_time)))

    def _update_gripper(self):
        """Maintains gripper position and telemetery (20Hz)."""
        if self.is_calibrated:
            self.gripper.move_to(self.target_gripper_pos)
            self.robot.push_command()
            self.pub_width.publish(Float32(data=float(self.gripper.status['pos'])))

    def shutdown(self):
        self.is_running = False
        self.get_logger().info("Disabling hardware controller...")
        self.robot.stop()

def main():
    rclpy.init()
    node = UMINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()
        os._exit(0)

if __name__ == '__main__':
    main()