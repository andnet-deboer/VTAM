#!/usr/bin/env python3
"""
imu_corrector.py — Delta-based backlash correction.

Compares how much the encoder vs IMU each changed since startup.
The difference is backlash error. Applied as a correction to the encoder.

Frame offsets cancel out because we only look at deltas.
"""

import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ImuCorrector:

    def __init__(self, node, imu_topic='/camera/camera/imu', n_samples=4):
        self.node = node
        self.ready = False

        # Baselines captured on first call
        self.imu_tilt_baseline = None
        self.enc_tilt_baseline = None
        self.imu_pan_baseline = 0.0
        self.enc_pan_baseline = None

        # Current IMU values (averaged)
        self.imu_tilt = None
        self.imu_pan_delta = 0.0  # integrated gyro change from baseline
        self.prev_time = None

        self.tilt_buf = deque(maxlen=n_samples)
        self.gyro_buf = deque(maxlen=n_samples)

        node.declare_parameter('imu_pan_decay', 0.998)
        self.pan_decay = node.get_parameter('imu_pan_decay').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Imu, imu_topic, self._imu_cb, qos)
        node.get_logger().info(f"ImuCorrector (delta mode) on '{imu_topic}'")

    def _imu_cb(self, msg: Imu):
        # Tilt from gravity
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z
        self.tilt_buf.append(np.arctan2(az, -ax))
        self.imu_tilt = np.mean(self.tilt_buf)

        # Pan from gyro Z (averaged to reject spikes)
        self.gyro_buf.append(msg.angular_velocity.z)
        gz_clean = np.mean(self.gyro_buf)

        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is not None:
            dt = now - self.prev_time
            if 0 < dt < 0.1:
                self.imu_pan_delta += gz_clean * dt
                self.imu_pan_delta *= self.pan_decay
        self.prev_time = now

        if not self.ready and self.imu_tilt is not None:
            self.ready = True
            self.node.get_logger().info("IMU data flowing.")

    def correct_rotation(self, R_base_cam: R_scipy) -> R_scipy:
        if not self.ready:
            return R_base_cam

        rpy = R_base_cam.as_euler('xyz')
        enc_tilt = rpy[1]
        enc_pan = rpy[2]

        # Capture baselines on first call
        if self.enc_tilt_baseline is None:
            self.enc_tilt_baseline = enc_tilt
            self.imu_tilt_baseline = self.imu_tilt
            self.enc_pan_baseline = enc_pan
            return R_base_cam

        # Tilt: how much each changed since baseline
        enc_tilt_delta = enc_tilt - self.enc_tilt_baseline
        imu_tilt_delta = self.imu_tilt - self.imu_tilt_baseline
        tilt_correction = imu_tilt_delta - enc_tilt_delta

        # Pan: encoder delta vs integrated gyro delta
        enc_pan_delta = enc_pan - self.enc_pan_baseline
        # pan_correction = self.imu_pan_delta - enc_pan_delta
        pan_correction = 0.0

        # Apply corrections
        rpy[1] += tilt_correction
        rpy[2] += pan_correction

        return R_scipy.from_euler('xyz', rpy)