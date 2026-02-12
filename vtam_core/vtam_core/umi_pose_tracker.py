#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import stretch_body.robot as rb
import numpy as np
import threading
import time

class HeadServo(Node):
    def __init__(self):
        super().__init__('head_servo')
        
        self.robot = rb.Robot()
        if not self.robot.startup():
            self.get_logger().error("Robot startup failed!")
            exit(1)
        
        # --- PID Gains ---
        self.kp = 2.0
        self.kd = 0.1
        self.deadzone = 0.008
        
        # --- Velocity limits (rad/s) ---
        self.max_vel = 1.5
        
        # --- Safety Limits ---
        self.pan_range = [-3.8, 1.5]
        self.tilt_range = [-1.5, 0.3]
        
        # --- State ---
        self.err_pan = 0.0
        self.err_tilt = 0.0
        self.prev_err_pan = 0.0
        self.prev_err_tilt = 0.0
        self.last_detection_time = 0.0
        self.lock = threading.Lock()
        
        # Subscribe to poses
        self.create_subscription(PoseArray, 'aruco_poses', self.pose_callback, 1)
        
        # Control loop in separate thread (100Hz)
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info("Head servo ready")

    def pose_callback(self, msg):
        """Just grab error, don't block"""
        if not msg.poses:
            return
        with self.lock:
            # Camera optical frame: X=right, Y=down, Z=forward
            # Camera is rotated 90° CW on Stretch head
            # 
            # Optical X (right in camera) → Up/Down in world → Tilt
            # Optical Y (down in camera) → Left/Right in world → Pan
            #
            # Flip signs as needed if direction is wrong
            self.err_pan = msg.poses[0].position.y   # Marker right → pan right (negative)
            self.err_tilt = -msg.poses[0].position.x  # Marker up → tilt up (negative)
            self.last_detection_time = time.time()

    def control_loop(self):
        """High-frequency control loop"""
        dt = 0.01  # 100Hz
        
        while self.running:
            loop_start = time.time()
            
            with self.lock:
                err_pan = self.err_pan
                err_tilt = self.err_tilt
                age = time.time() - self.last_detection_time
            
            # Stop if detection is stale (>0.5s)
            if age > 0.5:
                self.robot.head.get_joint('head_pan').set_velocity(0)
                self.robot.head.get_joint('head_tilt').set_velocity(0)
                self.robot.push_command()
                time.sleep(dt)
                continue
            
            # Deadzone
            if abs(err_pan) < self.deadzone:
                err_pan = 0.0
            if abs(err_tilt) < self.deadzone:
                err_tilt = 0.0
            
            # PD control
            d_err_pan = (err_pan - self.prev_err_pan) / dt
            d_err_tilt = (err_tilt - self.prev_err_tilt) / dt
            
            vel_pan = self.kp * err_pan + self.kd * d_err_pan
            vel_tilt = self.kp * err_tilt + self.kd * d_err_tilt
            
            # Clamp velocities
            vel_pan = np.clip(vel_pan, -self.max_vel, self.max_vel)
            vel_tilt = np.clip(vel_tilt, -self.max_vel, self.max_vel)
            
            # Safety: check limits
            curr_pan = self.robot.head.status['head_pan']['pos']
            curr_tilt = self.robot.head.status['head_tilt']['pos']
            
            if curr_pan <= self.pan_range[0] and vel_pan < 0:
                vel_pan = 0
            if curr_pan >= self.pan_range[1] and vel_pan > 0:
                vel_pan = 0
            if curr_tilt <= self.tilt_range[0] and vel_tilt < 0:
                vel_tilt = 0
            if curr_tilt >= self.tilt_range[1] and vel_tilt > 0:
                vel_tilt = 0
            
            # Command velocity
            self.robot.head.get_joint('head_pan').set_velocity(vel_pan)
            self.robot.head.get_joint('head_tilt').set_velocity(vel_tilt)
            self.robot.push_command()
            
            # Store for derivative
            self.prev_err_pan = err_pan
            self.prev_err_tilt = err_tilt
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def stop(self):
        self.running = False
        self.robot.head.get_joint('head_pan').set_velocity(0)
        self.robot.head.get_joint('head_tilt').set_velocity(0)
        self.robot.push_command()
        self.robot.stop()

def main():
    rclpy.init()
    node = HeadServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()