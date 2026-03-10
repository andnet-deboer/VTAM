#!/usr/bin/env python3
"""
Head tracker service — scans for UMI cube, locks on, then returns.
Auto-locates on startup, can be re-triggered via /locate_umi service.
"""

import time
import numpy as np
import rclpy
import hello_helpers.hello_misc as hm
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
import threading


class HeadTrackerService(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'head_tracker', 'head_tracker',
                          wait_for_first_pointcloud=False)

        # Parameters
        self.declare_parameter('k_pan', 0.001)
        self.declare_parameter('k_tilt', 0.001)
        self.declare_parameter('sign_pan', -.0)
        self.declare_parameter('sign_tilt', -1.0)
        self.declare_parameter('pan_deadzone', 0.04)
        self.declare_parameter('tilt_deadzone', 0.05)
        # self.declare_parameter('image_w', 640)
        # self.declare_parameter('image_h', 480)
        self.declare_parameter('image_w', 1280)
        self.declare_parameter('image_h', 720)
        self.declare_parameter('scan_speed', 0.05)
        self.declare_parameter('lock_threshold', 30.0)
        self.declare_parameter('lock_confirmations', 10)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_cb)

        # State
        self._latest_pixel = None
        self._last_seen = 0.0
        self._scan_direction = 1.0

        # Joint limits
        self.pan_range = (-3.8, 1.5)
        self.tilt_range = (-1.5, 0.4)

        # Pixel centroid from detector
        self.create_subscription(PointStamped, '/umi_cube/pixel_centroid',
                                 self._pixel_cb, 10)

        # Service
        self.create_service(Trigger, 'locate_umi', self._locate_cb)

        self._detector_shutdown = self.create_client(Trigger, '/umi_detector/shutdown')

        # Auto-locate on startup after a short delay for detector to start
        self.get_logger().info("Head tracker service ready. Auto-locating in 3s...")
        threading.Thread(target=self._auto_locate, daemon=True).start()

    # -- Parameter helpers --
    def _load_params(self):
        g = self.get_parameter
        self.k_pan = g('k_pan').value
        self.k_tilt = g('k_tilt').value
        self.sign_pan = g('sign_pan').value
        self.sign_tilt = g('sign_tilt').value
        self.pan_deadzone = g('pan_deadzone').value
        self.tilt_deadzone = g('tilt_deadzone').value
        self.image_w = g('image_w').value
        self.image_h = g('image_h').value
        self.scan_speed = g('scan_speed').value
        self.lock_threshold = g('lock_threshold').value
        self.lock_confirmations = g('lock_confirmations').value

    def _param_cb(self, params):
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
        self._load_params()
        return SetParametersResult(successful=True)

    # Pixel centroid callback
    def _pixel_cb(self, msg):
        self._latest_pixel = msg
        self._last_seen = time.time()

    # Auto-locate on startup
    def _auto_locate(self):
        time.sleep(3.0)
        self.get_logger().info("Auto-locating UMI cube...")
        success, message = self._do_locate()
        if success:
            self.get_logger().info(f"Auto-locate succeeded: {message}")
            self._shutdown_detector()
        else:
            self.get_logger().warn(f"Auto-locate failed: {message}")

    def _shutdown_detector(self):
        if self._detector_shutdown.wait_for_service(timeout_sec=2.0):
            self._detector_shutdown.call_async(Trigger.Request())
            self.get_logger().info("Sent shutdown to umi_detector_node.")
            
    # Service callback
    def _locate_cb(self, request, response):
        success, message = self._do_locate()
        response.success = success
        response.message = message
        return response

    # Core locate logic
    def _do_locate(self):
        self._latest_pixel = None
        confirmations = 0
        start = time.time()
        timeout = 30.0
        scan_direction = 1.0

        self.move_to_pose({'joint_head_tilt': -0.5}, blocking=True)

        while time.time() - start < timeout:
            now = time.time()

            if self._latest_pixel is not None and (now - self._last_seen) < 1.0:
                # Got a detection — servo toward it
                err_x = self._latest_pixel.point.x - self.image_w / 2.0
                err_y = self._latest_pixel.point.y - self.image_h / 2.0

                if abs(err_x) < self.lock_threshold and abs(err_y) < self.lock_threshold:
                    confirmations += 1
                    if confirmations >= self.lock_confirmations:
                        self.get_logger().info("UMI cube located and centered.")
                        return True, "Cube found and head locked."
                else:
                    confirmations = 0

                pan_delta = -err_y * self.k_pan * self.sign_pan
                tilt_delta = err_x * self.k_tilt * self.sign_tilt

                cmd = {}
                if abs(pan_delta) > self.pan_deadzone:
                    cmd['joint_head_pan'] = float(np.clip(
                        self._get_current_pan() + pan_delta, *self.pan_range))
                if abs(tilt_delta) > self.tilt_deadzone:
                    cmd['joint_head_tilt'] = float(np.clip(
                        self._get_current_tilt() + tilt_delta, *self.tilt_range))

                if cmd:
                    self.move_to_pose(cmd, blocking=False)

                time.sleep(0.033)
            else:
                # No detection — scan incrementally
                confirmations = 0
                curr_pan = self._get_current_pan()

                if scan_direction > 0 and curr_pan >= self.pan_range[1] - 0.2:
                    scan_direction = -1.0
                elif scan_direction < 0 and curr_pan <= self.pan_range[0] + 0.2:
                    scan_direction = 1.0

                target = float(np.clip(
                    curr_pan + self.scan_speed * scan_direction, *self.pan_range))

                self.move_to_pose({'joint_head_pan': target}, blocking=False)
                time.sleep(0.05)

        self.get_logger().warn("Timeout — cube not found.")
        return False, "Cube not found within timeout."

    # -- Joint state helpers --
    def _get_current_pan(self):
        try:
            idx = self.joint_state.name.index('joint_head_pan')
            return self.joint_state.position[idx]
        except:
            return 0.0

    def _get_current_tilt(self):
        try:
            idx = self.joint_state.name.index('joint_head_tilt')
            return self.joint_state.position[idx]
        except:
            return -0.5

    # -- Scan step --

    def _do_scan_step(self):
        curr_pan = self._get_current_pan()

        if curr_pan >= self.pan_range[1] - 0.1:
            self._scan_direction = -1.0
        elif curr_pan <= self.pan_range[0] + 0.1:
            self._scan_direction = 1.0

        target_pan = float(np.clip(
            curr_pan + self.scan_speed * self._scan_direction, *self.pan_range))

        self.move_to_pose({'joint_head_pan': target_pan}, blocking=False)


def main():
    node = HeadTrackerService()
    node.main()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()