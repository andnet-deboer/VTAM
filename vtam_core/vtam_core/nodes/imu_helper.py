#!/usr/bin/env python3
"""
imu_calibration_helper.py — Run this to figure out your axis mapping.

Usage:
    ros2 run vtam_core imu_calibration_helper

Then manually pan/tilt the head and watch the output.
It will tell you which gyro axis lights up for each motion.

Make sure your realsense launch includes:
    unite_imu_method:=2  enable_accel:=true  enable_gyro:=true
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np


class ImuCalibHelper(Node):
    def __init__(self):
        super().__init__('imu_calib_helper')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/camera/imu', self.imu_cb, qos)
        self.frame_count = 0
        self.get_logger().info(
            "\n"
            "============================================\n"
            "  D435i IMU Calibration Helper\n"
            "============================================\n"
            "\n"
            "Step 1: Keep the head STILL for 3 seconds.\n"
            "        Note the gravity vector (accel).\n"
            "\n"
            "Step 2: Slowly PAN the head left/right.\n"
            "        Note which gyro axis spikes.\n"
            "\n"
            "Step 3: Slowly TILT the head up/down.\n"
            "        Note which accel values change.\n"
            "\n"
            "============================================\n"
        )

    def imu_cb(self, msg: Imu):
        self.frame_count += 1
        if self.frame_count % 15 != 0:  # Print ~every 0.5s at 30Hz
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Compute tilt from gravity
        tilt_rad = np.arctan2(-az, ay)
        tilt_deg = np.degrees(tilt_rad)

        # Find dominant gyro axis
        gyro_vals = {'x': gx, 'y': gy, 'z': gz}
        dominant = max(gyro_vals, key=lambda k: abs(gyro_vals[k]))
        dominant_val = gyro_vals[dominant]

        # Determine if stationary or moving
        total_gyro = np.sqrt(gx**2 + gy**2 + gz**2)
        status = "MOVING" if total_gyro > 0.1 else "still "

        self.get_logger().info(
            f"[{status}] "
            f"Accel: x={ax:+6.2f} y={ay:+6.2f} z={az:+6.2f} | "
            f"Gyro: x={gx:+5.2f} y={gy:+5.2f} z={gz:+5.2f} | "
            f"Tilt={tilt_deg:+6.1f}° | "
            f"Dominant gyro: {dominant}={dominant_val:+.2f}"
        )


def main():
    rclpy.init()
    node = ImuCalibHelper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()