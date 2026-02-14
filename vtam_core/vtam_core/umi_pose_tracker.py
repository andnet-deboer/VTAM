import math
import time
import numpy as np
import rclpy
import csv
import os
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class HeadTrackerNode:
    """
    Head tracking behavior designed to run inside a parent RobotNode.
    Logs tracking data to a CSV file for debugging.
    """
    def __init__(self, node, robot):
        self.node = node
        self.robot = robot
        self.logger = node.get_logger()
        
        # Reuse the TF buffer from the parent RobotNode
        if hasattr(node, 'tf_buffer'):
            self.tf_buffer = node.tf_buffer
        else:
            self.logger.error("HeadTracker: Parent node missing 'tf_buffer'. Tracking will fail.")
            self.tf_buffer = None

        # ─── Parameters ───
        if not node.has_parameter('head_kp'):
            node.declare_parameter('head_kp', 1.5)
            node.declare_parameter('head_kd', 0.05)
            node.declare_parameter('head_max_vel', 1.0)
            node.declare_parameter('head_target_frame', 'umi_gripper')
            node.declare_parameter('head_camera_frame', 'camera_color_optical_frame')
            node.declare_parameter('head_log_file', 'head_tracker_log.csv')

        self.kp = node.get_parameter('head_kp').value
        self.kd = node.get_parameter('head_kd').value
        self.max_vel = node.get_parameter('head_max_vel').value
        self.target_frame = node.get_parameter('head_target_frame').value
        self.camera_frame = node.get_parameter('head_camera_frame').value
        self.log_file_path = node.get_parameter('head_log_file').value

        # Callback for dynamic parameter tuning
        self.node.add_on_set_parameters_callback(self._param_cb)

        # ─── Data Logging Setup ───
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        # Header
        self.csv_writer.writerow([
            'timestamp', 
            'error_pan', 'error_tilt', 
            'cmd_vel_pan', 'cmd_vel_tilt',
            'cam_x', 'cam_y', 'cam_z', 'cam_qx', 'cam_qy', 'cam_qz', 'cam_qw',
            'tgt_x', 'tgt_y', 'tgt_z', 'tgt_qx', 'tgt_qy', 'tgt_qz', 'tgt_qw'
        ])
        self.logger.info(f"Logging head tracking data to: {os.path.abspath(self.log_file_path)}")

        # ─── State ───
        self.last_time = time.time()
        self.prev_err_pan = 0.0
        self.prev_err_tilt = 0.0
        
        # Scan State
        self.scan_mode = False
        self.scan_dir = -1  # -1 = Down, 1 = Up
        self.scan_vel = 0.2
        
        # Safety Limits (Stretch RE1 Head)
        self.pan_range = [-3.8, 1.5]
        self.tilt_range = [-1.5, 0.4]

    def _param_cb(self, params):
        for p in params:
            if p.name == 'head_kp': self.kp = p.value
            if p.name == 'head_kd': self.kd = p.value
            if p.name == 'head_max_vel': self.max_vel = p.value
            self.logger.info(f"Param updated: {p.name} = {p.value}")
        return SetParametersResult(successful=True)

    def tick(self):
        """
        Called 100Hz by parent node. 
        Calculates error -> Updates velocities -> Logs Data.
        """
        if self.tf_buffer is None:
            return

        # 1. Calculate DT
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0 or dt > 0.1:
            dt = 0.01

        # 2. Get Tracking Error
        err_pan, err_tilt = self._get_error()

        # 3. Control Logic
        vel_pan = 0.0
        vel_tilt = 0.0

        if err_pan is None:
            # ─── LOST TARGET -> SCAN ───
            if not self.scan_mode:
                self.logger.warn(">> TARGET LOST. Entering SCAN MODE.")
                self.scan_mode = True
            
            try:
                curr_tilt = self.robot.head.status['head_tilt']['pos']
            except KeyError:
                return
            
            # Switch directions at limits
            if curr_tilt <= -1.1: self.scan_dir = 1   # Go Up
            if curr_tilt >= -0.1: self.scan_dir = -1  # Go Down
            
            vel_tilt = self.scan_vel * self.scan_dir
            vel_pan = 0.0
            
            self.prev_err_pan = 0.0
            self.prev_err_tilt = 0.0

        else:
            # ─── TRACKING -> PD CONTROL ───
            if self.scan_mode:
                self.logger.info(f">> TARGET FOUND. Error: P={math.degrees(err_pan):.1f}deg T={math.degrees(err_tilt):.1f}deg")
                self.scan_mode = False
            
            d_pan = (err_pan - self.prev_err_pan) / dt
            d_tilt = (err_tilt - self.prev_err_tilt) / dt

            # Gain Logic: -1.0 * (Kp*err + Kd*d_err)
            raw_v_pan = -1.0 * (self.kp * err_pan + self.kd * d_pan)
            raw_v_tilt = -1.0 * (self.kp * err_tilt + self.kd * d_tilt)
            
            vel_pan = raw_v_pan
            vel_tilt = raw_v_tilt

            # Deadzone
            if abs(err_pan) < 0.03: vel_pan = 0.0
            if abs(err_tilt) < 0.03: vel_tilt = 0.0

            # DEBUG: Log Direction
            p_action = "LEFT" if vel_pan > 0 else "RIGHT"
            t_action = "UP" if vel_tilt > 0 else "DOWN"
            
            self.logger.info(
                f"TRACKING | Err P/T: {err_pan:.3f}/{err_tilt:.3f} | "
                f"Cmd: {vel_pan:.2f} ({p_action}) / {vel_tilt:.2f} ({t_action})",
                throttle_duration_sec=0.5
            )

            self.prev_err_pan = err_pan
            self.prev_err_tilt = err_tilt

        # 4. Clamp Velocities
        vel_pan = np.clip(vel_pan, -self.max_vel, self.max_vel)
        vel_tilt = np.clip(vel_tilt, -self.max_vel, self.max_vel)

        # 5. Soft Joint Limits
        try:
            curr_pan = self.robot.head.status['head_pan']['pos']
            curr_tilt = self.robot.head.status['head_tilt']['pos']
        except KeyError:
            return

        if curr_pan <= self.pan_range[0] and vel_pan < 0: 
            vel_pan = 0.0
        if curr_pan >= self.pan_range[1] and vel_pan > 0: 
            vel_pan = 0.0
        
        if curr_tilt <= self.tilt_range[0] and vel_tilt < 0: 
            vel_tilt = 0.0
        if curr_tilt >= self.tilt_range[1] and vel_tilt > 0: 
            vel_tilt = 0.0

        # 6. Set Commands
        self.robot.head.get_joint('head_pan').set_velocity(vel_pan)
        self.robot.head.get_joint('head_tilt').set_velocity(vel_tilt)

        # 7. Write to Log File
        self._log_state(err_pan, err_tilt, vel_pan, vel_tilt)

    def _get_error(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame, 
                self.target_frame,
                rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            if z < 0.1: 
                return None, None

            err_pan = math.atan2(x, z) 
            err_tilt = math.atan2(y, z)

            return err_pan, err_tilt

        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None

    def _log_state(self, err_pan, err_tilt, cmd_pan, cmd_tilt):
        """Writes current state and world transforms to CSV."""
        try:
            # 1. Get World Transforms (base_link frame)
            # You can change 'base_link' to 'map' or 'odom' if available
            world_frame = 'base_link' 
            
            # Camera Transform in World
            t_cam = self.tf_buffer.lookup_transform(
                world_frame, self.camera_frame, rclpy.time.Time())
            
            # Target Transform in World (if visible)
            try:
                t_tgt = self.tf_buffer.lookup_transform(
                    world_frame, self.target_frame, rclpy.time.Time())
                tgt_vals = [
                    t_tgt.transform.translation.x, t_tgt.transform.translation.y, t_tgt.transform.translation.z,
                    t_tgt.transform.rotation.x, t_tgt.transform.rotation.y, t_tgt.transform.rotation.z, t_tgt.transform.rotation.w
                ]
            except (LookupException, ConnectivityException, ExtrapolationException):
                tgt_vals = [0.0] * 7

            # Prepare Row
            row = [
                time.time(),
                err_pan if err_pan is not None else 0.0,
                err_tilt if err_tilt is not None else 0.0,
                cmd_pan,
                cmd_tilt,
                # Camera Pose
                t_cam.transform.translation.x, t_cam.transform.translation.y, t_cam.transform.translation.z,
                t_cam.transform.rotation.x, t_cam.transform.rotation.y, t_cam.transform.rotation.z, t_cam.transform.rotation.w,
                # Target Pose
                *tgt_vals
            ]
            
            self.csv_writer.writerow(row)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass # Skip logging if base_link transform fails
    
    def __del__(self):
        if hasattr(self, 'log_file'):
            self.log_file.close()