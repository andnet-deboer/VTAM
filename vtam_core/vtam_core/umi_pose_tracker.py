#!/usr/bin/env python3
"""
head_pose_tracker.py — Visual servoing head tracker for the UMI cube.

Subscribes to /umi_cube/pixel_centroid (PointStamped) published by
umi_detector_node and drives head_pan / head_tilt to keep the cube
centred in the D435i image using a simple P controller.

Camera is mounted VERTICALLY so image axes are swapped vs world:
    image X (columns) → real-world vertical   → drives TILT
    image Y (rows)    → real-world horizontal  → drives PAN

Control law:
    err_x   = centroid_px - image_w / 2
    err_y   = centroid_py - image_h / 2
    pan_cmd  = current_pan  - err_y * k_pan  * sign_pan
    tilt_cmd = current_tilt + err_x * k_tilt * sign_tilt

Dynamic reconfigure (ros2 param set):
    k_pan        float   Gain for pan  axis (rad/px). Default 0.001
    k_tilt       float   Gain for tilt axis (rad/px). Default 0.001
    sign_pan     float   +1.0 or -1.0 — flip pan  direction
    sign_tilt    float   +1.0 or -1.0 — flip tilt direction
    pan_deadzone float   Min pan  change before issuing command (rad)
    tilt_deadzone float  Min tilt change before issuing command (rad)
    image_w      int     Image width  (pixels) — matches D435i resolution
    image_h      int     Image height (pixels) — matches D435i resolution

Example:
    ros2 param set /your_node_name sign_pan -1.0
    ros2 param set /your_node_name k_pan 0.002
"""
from std_msgs.msg import Bool
import time
import numpy as np

from rcl_interfaces.msg import (
    ParameterDescriptor, ParameterType, SetParametersResult
)
from geometry_msgs.msg import PointStamped


class HeadPoseTracker:
    """
    Instantiate inside your ROS2 node:

        self.head_tracker = HeadPoseTracker(self, robot)

    Then call in your control loop timer:

        self.head_tracker.tick()
    """

    def __init__(self, node, robot):
        self.node  = node
        self.robot = robot

        # ── Declare ROS parameters ────────────────────────────────────────────
        node.declare_parameter('k_pan',         0.001,
            ParameterDescriptor(description='Pan gain (rad/px). Increase if sluggish, decrease if oscillating.'))
        node.declare_parameter('k_tilt',        0.001,
            ParameterDescriptor(description='Tilt gain (rad/px).'))
        node.declare_parameter('sign_pan',       -1.0,
            ParameterDescriptor(description='Flip pan direction: +1.0 or -1.0'))
        node.declare_parameter('sign_tilt',      -1.0,
            ParameterDescriptor(description='Flip tilt direction: +1.0 or -1.0'))
        node.declare_parameter('pan_deadzone',   0.04,
            ParameterDescriptor(description='Min pan  delta before issuing move_to (rad)'))
        node.declare_parameter('tilt_deadzone',  0.05,
            ParameterDescriptor(description='Min tilt delta before issuing move_to (rad)'))
        node.declare_parameter('image_w',        640,
            ParameterDescriptor(description='D435i image width  (pixels)'))
        node.declare_parameter('image_h',        480,
            ParameterDescriptor(description='D435i image height (pixels)'))

        self._load_params()
        node.add_on_set_parameters_callback(self._param_cb)

        self.recording = False

        # ── Motion profiles ───────────────────────────────────────────────────
        self.pan_profile  = {'v': 4.0,  'a': 5.0}
        self.tilt_profile = {'v': 2.0,  'a': 3.0}

        # ── Joint limits ──────────────────────────────────────────────────────
        self.pan_range  = (-3.8, 1.5)
        self.tilt_range = (-1.5, 0.4)

        # ── Scan state ────────────────────────────────────────────────────────
        self.last_seen      = 0.0
        self.scanning       = False
        self.has_locked = False
        self.scan_direction = 1.0   # +1 CCW, -1 CW

        # ── Latest pixel centroid ─────────────────────────────────────────────
        self._latest_pixel: PointStamped | None = None

        node.create_subscription(
            PointStamped,
            '/umi_cube/pixel_centroid',
            self._pixel_cb,
            10
        )

        node.create_subscription(Bool, '/recording/active', 
    lambda m: setattr(self, 'recording', m.data), 10)

        node.get_logger().info(
            "HeadPoseTracker ready.\n"
            f"  k_pan={self.k_pan}  sign_pan={self.sign_pan}\n"
            f"  k_tilt={self.k_tilt}  sign_tilt={self.sign_tilt}\n"
            f"  image: {self.image_w}x{self.image_h}\n"
            "Tune signs with:\n"
            "  ros2 param set /<node> sign_pan  -1.0\n"
            "  ros2 param set /<node> sign_tilt -1.0"
        )

    # ── Parameter helpers ─────────────────────────────────────────────────────

    def _load_params(self):
        g = self.node.get_parameter
        self.k_pan         = g('k_pan').value
        self.k_tilt        = g('k_tilt').value
        self.sign_pan      = g('sign_pan').value
        self.sign_tilt     = g('sign_tilt').value
        self.pan_deadzone  = g('pan_deadzone').value
        self.tilt_deadzone = g('tilt_deadzone').value
        self.image_w       = g('image_w').value
        self.image_h       = g('image_h').value

    def _param_cb(self, params):
        for p in params:
            if p.name == 'k_pan':          self.k_pan         = p.value
            if p.name == 'k_tilt':         self.k_tilt        = p.value
            if p.name == 'sign_pan':       self.sign_pan      = p.value
            if p.name == 'sign_tilt':      self.sign_tilt     = p.value
            if p.name == 'pan_deadzone':   self.pan_deadzone  = p.value
            if p.name == 'tilt_deadzone':  self.tilt_deadzone = p.value
            if p.name == 'image_w':        self.image_w       = p.value
            if p.name == 'image_h':        self.image_h       = p.value
        self.node.get_logger().info(
            f"Params updated — k_pan={self.k_pan} sign_pan={self.sign_pan} "
            f"k_tilt={self.k_tilt} sign_tilt={self.sign_tilt}"
        )
        return SetParametersResult(successful=True)

    # ── Pixel centroid callback ───────────────────────────────────────────────

    def _pixel_cb(self, msg: PointStamped):
        self._latest_pixel = msg
        self.last_seen = time.time()

    # ── Main tick ─────────────────────────────────────────────────────────────

    def tick(self):
        if self.has_locked:
            return
        now = time.time()

        if self._latest_pixel is None or (now - self.last_seen) > 2.0:
            self._scan()
            return

        # Stop scan if we were scanning
        if self.scanning:
            self.node.get_logger().info("Target locked — stopping scan.")
            self.robot.head.get_joint('head_pan').set_velocity(0.0)
            self.scanning = False
            self.has_locked = True
            return    

        # Always read TRUE current joint positions — avoids snap-to-home bug
        try:
            current_pan  = self.robot.head.status['head_pan']['pos']
            current_tilt = self.robot.head.status['head_tilt']['pos']
        except Exception as e:
            self.node.get_logger().warn(f"Could not read head joint state: {e}")
            return

        # Pixel error from image centre
        err_x = self._latest_pixel.point.x - self.image_w / 2.0
        err_y = self._latest_pixel.point.y - self.image_h / 2.0

        # Camera mounted VERTICALLY:
        #   image X (cols) → world vertical   → tilt
        #   image Y (rows) → world horizontal → pan
        pan_cmd  = current_pan  - err_y * self.k_pan  * self.sign_pan
        tilt_cmd = current_tilt + err_x * self.k_tilt * self.sign_tilt

        pan_cmd  = float(np.clip(pan_cmd,  *self.pan_range))
        tilt_cmd = float(np.clip(tilt_cmd, *self.tilt_range))

        # Deadzone gate — suppress hunting on sub-pixel noise
        if abs(pan_cmd - current_pan) > self.pan_deadzone:
            self.robot.head.move_to('head_pan',  pan_cmd,
                                    v_r=self.pan_profile['v'],
                                    a_r=self.pan_profile['a'])

        if abs(tilt_cmd - current_tilt) > self.tilt_deadzone:
            self.robot.head.move_to('head_tilt', tilt_cmd,
                                    v_r=self.tilt_profile['v'],
                                    a_r=self.tilt_profile['a'])

    # ── Scan ──────────────────────────────────────────────────────────────────

    def _scan(self):
        if not self.scanning:
            self.node.get_logger().info("No target — starting scan.")
            self.scanning = True
        try:
            curr_pan = self.robot.head.status['head_pan']['pos']

            if curr_pan >= self.pan_range[1] - 0.1:
                self.scan_direction = -1.0
            elif curr_pan <= self.pan_range[0] + 0.1:
                self.scan_direction = 1.0

            self.robot.head.get_joint('head_pan').set_velocity(0.6 * self.scan_direction)
            self.robot.head.move_to('head_tilt', -0.5)

        except Exception as e:
            self.node.get_logger().error(f"Scan error: {e}")