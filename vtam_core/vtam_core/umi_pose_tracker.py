#!/usr/bin/env python3
import math
import time
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration

class AlphaBetaFilter:
    """Estimates position and velocity with noise-gate deadband logic."""
    def __init__(self, alpha=0.8, beta=0.2, deadband=0.005):
        self.alpha = alpha
        self.beta = beta
        self.deadband = deadband 
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.initialized = False

    def update(self, measurement, dt):
        m = np.array(measurement)
        if not self.initialized:
            self.pos = m
            self.initialized = True
            return self.pos

        dt = max(dt, 0.001)
        dist_moved = np.linalg.norm(m - self.pos)
        
        if dist_moved < self.deadband:
            self.vel *= 0.1 
            return self.pos

        prediction = self.pos + (self.vel * dt)
        residual = m - prediction
        self.pos = prediction + (self.alpha * residual)
        self.vel = self.vel + (self.beta / dt) * residual
        return self.pos

class HeadPoseTracker:
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        
        # Motion Profiles
        self.pan_profile = {'v': 8.0, 'a': 35.0}  
        self.tilt_profile = {'v': 4.0, 'a': 15.0} 
        
        # Asymmetric Filtering
        self.pan_filter = AlphaBetaFilter(alpha=0.85, beta=0.3, deadband=0.004)
        self.tilt_filter = AlphaBetaFilter(alpha=0.15, beta=0.05, deadband=0.008)

        # Limits & State
        self.pan_range = (-3.8, 1.5)
        self.look_ahead = 0.06
        self.last_time = time.time()
        
        # Start scanning immediately by setting last_seen to the past
        self.last_seen = 0.0 
        self.scanning = False
        self.scan_direction = 1.0 # 1.0 for CCW, -1.0 for CW

    def tick(self):
        target = self._lookup_target()
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if target:
            self.last_seen = now
            self.pan_filter.update(target, dt)
            self.tilt_filter.update(target, dt)
            
            p_future = self.pan_filter.pos + (self.pan_filter.vel * self.look_ahead)
            t_future = self.tilt_filter.pos + (self.tilt_filter.vel * self.look_ahead)
            
            self._execute_move(p_future, t_future)
        elif now - self.last_seen > 2.0:
            self._scan()

    def _execute_move(self, p_target, t_target):
        if self.scanning:
            self.node.get_logger().info("Target Locked: Ceasing Scan")
            self.robot.head.get_joint('head_pan').set_velocity(0.0)
            self.scanning = False

        desired_pan = math.atan2(-p_target[0], p_target[1])
        horiz_dist = math.sqrt(t_target[0]**2 + t_target[1]**2)
        desired_tilt = math.atan2(t_target[2], horiz_dist)

        self.robot.head.move_to('head_pan', np.clip(desired_pan, *self.pan_range), 
                                v_r=self.pan_profile['v'], a_r=self.pan_profile['a'])
        self.robot.head.move_to('head_tilt', np.clip(desired_tilt, -1.5, 0.4), 
                                v_r=self.tilt_profile['v'], a_r=self.tilt_profile['a'])

    def _lookup_target(self):
        try:
            t = self.node.tf_buffer.lookup_transform(
                'link_head', 'umi_gripper', Time(seconds=0), timeout=Duration(seconds=0.01)
            )
            tr = t.transform.translation
            return (tr.x, tr.y, tr.z)
        except Exception:
            return None

    def _scan(self):
        """Sweeps the head back and forth across the pan range."""
        if not self.scanning:
            self.node.get_logger().info("Starting Scan")
            self.scanning = True
        
        try:
            # Get current position to check for boundary hit
            curr_pan = self.robot.head.status['head_pan']['pos']
            
            # Switch direction if near limits
            # Margin of 0.1 radians to prevent getting stuck at the edge
            if curr_pan >= self.pan_range[1] - 0.1:
                self.scan_direction = -1.0
            elif curr_pan <= self.pan_range[0] + 0.1:
                self.scan_direction = 1.0
            
            # Set sweep velocity
            sweep_vel = 0.6 * self.scan_direction
            
            self.robot.head.get_joint('head_pan').set_velocity(sweep_vel)
            # Keep head slightly tilted down to find objects on tables
            self.robot.head.move_to('head_tilt', -0.5)
            
        except Exception as e:
            self.node.get_logger().error(f"Scan failed: {e}")