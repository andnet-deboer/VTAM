#!/usr/bin/env python3
import math
import time
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Vector3Stamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class AlphaBetaFilter:
    """Estimates position and velocity to predict future target state."""
    def __init__(self, alpha=0.8, beta=0.2):
        self.alpha = alpha
        self.beta = beta
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.initialized = False

    def update(self, measurement, dt):
        if not self.initialized:
            self.pos = np.array(measurement)
            self.initialized = True
            return self.pos
        dt = max(dt, 0.001)
        predicted_pos = self.pos + (self.vel * dt)
        residual = np.array(measurement) - predicted_pos
        self.pos = predicted_pos + (self.alpha * residual)
        self.vel = self.vel + (self.beta / dt) * residual
        return self.pos

class HeadPoseTracker:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        
        # Motion Profiles
        self.pan_profile = {'v': 8.0, 'a': 30.0}   # High velocity/accel for Pan
        self.tilt_profile = {'v': 4.0, 'a': 15.0}  # Damped for Tilt stability
        
        # Asymmetric filters to prioritize pan responsiveness while keeping tilt stable
        # High alpha for pan to stay inside vertical FOV
        self.pan_filter = AlphaBetaFilter(alpha=0.85, beta=0.3)
        self.tilt_filter = AlphaBetaFilter(alpha=0.4, beta=0.1)

        self.look_ahead = 0.05  # 50ms latency compensation
        self.last_time = time.time()
        self.last_seen = time.time()
        self.scanning = False

    def tick(self):
        target = self._lookup_target()
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if target:
            self.last_seen = now
            # Update state estimators
            self.pan_filter.update(target, dt)
            self.tilt_filter.update(target, dt)
            
            # Predict lead-position
            p_future = self.pan_filter.pos + (self.pan_filter.vel * self.look_ahead)
            t_future = self.tilt_filter.pos + (self.tilt_filter.vel * self.look_ahead)
            
            self._execute_move(p_future, t_future)
        elif now - self.last_seen > 2.0:
            self._scan()

    def _execute_move(self, p_target, t_target):
        if self.scanning:
            self.node.get_logger().info("Target Re-acquired")
            self.scanning = False

        # Calculate target angles
        desired_pan = math.atan2(-p_target[0], p_target[1])
        horiz = math.sqrt(t_target[0]**2 + t_target[1]**2)
        desired_tilt = math.atan2(t_target[2], horiz)

        # Apply Physical Safety Limits
        p_cmd = np.clip(desired_pan, -3.8, 1.5)
        t_cmd = np.clip(desired_tilt, -1.5, 0.4)

        # Passing velocity and acceleration
        self.robot.head.move_to('head_pan', p_cmd, 
                                v_r=self.pan_profile['v'], 
                                a_r=self.pan_profile['a'])
        self.robot.head.move_to('head_tilt', t_cmd, 
                                v_r=self.tilt_profile['v'], 
                                a_r=self.tilt_profile['a'])

    def _lookup_target(self):
        try:
            # Get the most recent valid transform
            t = self.node.tf_buffer.lookup_transform(
                'link_head', 'fiducial_cube', Time(seconds=0), timeout=Duration(seconds=0.01)
            )
            tr = t.transform.translation
            return (tr.x, tr.y, tr.z)
        except Exception:
            return None

    def _scan(self):
        self.scanning = True
        try:
            # Simple constant-velocity sweep
            self.robot.head.get_joint('head_pan').set_velocity(0.8)
            self.robot.head.move_to('head_tilt', -0.4)
        except Exception:
            pass