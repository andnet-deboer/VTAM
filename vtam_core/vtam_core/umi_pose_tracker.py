#!/usr/bin/env python3
"""
Head Pose Tracker - direct angle computation.
Looks up target in link_head frame, computes pan/tilt, calls move_to().

link_head frame: X=right, Y=forward, Z=up
  pan  = atan2(-x, y)       positive pan = look left
  tilt = atan2(z, horiz)    negative z = below = look down

Scan mode: if target lost for scan_timeout, sweeps pan with tilt flat.
"""

import math
import time
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class HeadPoseTracker:

    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        self.logger = node.get_logger()
        self.tf_buffer = getattr(node, 'tf_buffer', None)

        self.head_frame = 'link_head'
        self.target_frame = 'fiducial_cube'

        defaults = {
            'head_scan_timeout': 2.0,
            'head_scan_vel': 0.5,
            'head_scan_tilt': -0.5,
        }
        for name, val in defaults.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, val)

        self._load_params()
        node.add_on_set_parameters_callback(self._param_cb)

        self.scanning = False
        self.last_seen = time.time()
        self.scan_dir = 1

        self.pan_range = (-3.8, 1.5)
        self.tilt_range = (-1.5, 0.4)

        self.pub_error = node.create_publisher(Vector3Stamped, '/head_tracker/error', 10)
        self.startup_time = node.get_clock().now()

        self.logger.info("HeadPoseTracker ready")

    def _load_params(self):
        self.scan_timeout = self.node.get_parameter('head_scan_timeout').value
        self.scan_vel = self.node.get_parameter('head_scan_vel').value
        self.scan_tilt = self.node.get_parameter('head_scan_tilt').value

    def _param_cb(self, params):
        self._load_params()
        return SetParametersResult(successful=True)

    def tick(self):
        if self.tf_buffer is None:
            return

        target = self._lookup_target()

        if target is not None:
            self.last_seen = time.time()
            self._track(target)
        elif time.time() - self.last_seen > self.scan_timeout:
            self._scan()

    def _track(self, target):
        if self.scanning:
            self.logger.info("Target acquired")
            self.scanning = False

        x, y, z = target

        # link_head: X=right, Y=forward, Z=up
        desired_pan = math.atan2(-x, y)
        horiz = math.sqrt(x**2 + y**2)
        desired_tilt = math.atan2(z, horiz)

        desired_pan = np.clip(desired_pan, *self.pan_range)
        desired_tilt = np.clip(desired_tilt, *self.tilt_range)

        self.robot.head.get_joint('head_pan').move_to(desired_pan)
        self.robot.head.get_joint('head_tilt').move_to(desired_tilt)

        msg = Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.head_frame
        msg.vector.x = desired_pan
        msg.vector.y = desired_tilt
        msg.vector.z = horiz
        self.pub_error.publish(msg)

        self.logger.info(
            f"TRACK pan={math.degrees(desired_pan):+.1f} tilt={math.degrees(desired_tilt):+.1f} "
            f"dist={horiz:.2f}m",
            throttle_duration_sec=0.5
        )

    def _scan(self):
        if not self.scanning:
            self.logger.info("Target lost, scanning")
            self.scanning = True

        try:
            curr_pan = self.robot.head.status['head_pan']['pos']
        except KeyError:
            return

        margin = 0.1
        if curr_pan <= self.pan_range[0] + margin:
            self.scan_dir = 1
        elif curr_pan >= self.pan_range[1] - margin:
            self.scan_dir = -1

        self.robot.head.get_joint('head_pan').set_velocity(self.scan_vel * self.scan_dir)
        self.robot.head.get_joint('head_tilt').move_to(self.scan_tilt)

    def _lookup_target(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.head_frame, self.target_frame,
                Time(seconds=0, nanoseconds=0),
                timeout=Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            dist = math.sqrt(x**2 + y**2 + z**2)
            if dist < 0.05 or dist > 5.0:
                return None

            return (x, y, z)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            elapsed = (self.node.get_clock().now() - self.startup_time).nanoseconds / 1e9
            if elapsed > 5.0:
                self.logger.warn(f"TF lookup failed: {e}", throttle_duration_sec=5.0)
            return None