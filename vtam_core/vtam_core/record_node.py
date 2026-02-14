#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal
from datetime import datetime
import time

class RecordDemoNode(Node):
    def __init__(self):
        super().__init__('record_demo_node')
        self.declare_parameter('demo_name', 'demo')
        
        self.recording_process = None
        self.current_bag_path = None
        self.save_dir = os.path.expanduser("~/VTAM/data/raw")
        os.makedirs(self.save_dir, exist_ok=True)

        # Safety Parameters
        self.max_size_gb = 10.0
        self.max_bytes = self.max_size_gb * 1024 * 1024 * 1024
        
        # Priority Watchdog - Monitors disk usage and process health
        self.watchdog_timer = self.create_timer(1.0, self.storage_watchdog)

        self.srv = self.create_service(SetBool, 'record_demo', self.record_callback)
        self.get_logger().info(f'✓ PRIORITY Recorder Active (Limit: {self.max_size_gb}GB)')

    def get_dir_size(self, path):
        total_size = 0
        if not os.path.exists(path):
            return 0
        for dirpath, _, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)
        return total_size

    def stop_recording(self):
        """High-priority interrupt to kill the recording process group."""
        if self.recording_process is not None:
            self.get_logger().warn("INTERRUPT: Shutting down recording process...")
            try:
                # Get the process group ID
                pgid = os.getpgid(self.recording_process.pid)
                
                # Step 1: SIGINT (Friendly stop for MCAP header writing)
                os.killpg(pgid, signal.SIGINT)
                
                # Step 2: Short wait/check
                start_wait = time.time()
                while time.time() - start_wait < 0.5:
                    if self.recording_process.poll() is not None:
                        break
                    time.sleep(0.1)
                
                # Step 3: SIGKILL (The hammer - if still running)
                if self.recording_process.poll() is None:
                    self.get_logger().error("Process hung. Escalating to SIGKILL.")
                    os.killpg(pgid, signal.SIGKILL)
                
            except Exception as e:
                self.get_logger().error(f"Error during interrupt: {e}")
            finally:
                # ABSOLUTE RESET: Allow immediate restart
                self.recording_process = None
                self.current_bag_path = None
                self.get_logger().info("✓ System State Reset.")

    def storage_watchdog(self):
        if self.recording_process and self.current_bag_path:
            current_size = self.get_dir_size(self.current_bag_path)
            if current_size > self.max_bytes:
                self.get_logger().error("DISK LIMIT EXCEEDED. TRIGGERING STOP.")
                self.stop_recording()

    def record_callback(self, request, response):
            # 1. Check if the process exists and is actually still running
            is_running = (self.recording_process is not None and 
                        self.recording_process.poll() is None)

            if not is_running:
                # Clean up stale process object if it crashed
                if self.recording_process is not None:
                    self.recording_process = None

                # --- START RECORDING ---
                name_prefix = self.get_parameter('demo_name').get_parameter_value().string_value
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.current_bag_path = os.path.join(self.save_dir, f"{name_prefix}_{timestamp}")

                topics = [
                    "/joint_states", "/tf", "/tf_static", "/umi_cube_pose",
                    "/camera/color/image_raw/compressed", "/camera/color/camera_info",
                    "/camera_arm/color/image_rect_raw/compressed", "/camera_arm/color/camera_info",
                    "/any_skin/data"
                ]

                # Use standard compression flags for stability
                cmd = [
                    "ros2", "bag", "record", "-s", "mcap",
                    "-o", self.current_bag_path,
                    "--compression-mode", "file",
                    "--compression-format", "zstd",
                ] + topics
                
                try:
                    self.recording_process = subprocess.Popen(cmd, start_new_session=True)
                    response.success = True
                    response.message = f"RECORDING START: {self.current_bag_path}"
                except Exception as e:
                    response.success = False
                    response.message = f"Failed to start: {str(e)}"
            else:
                # --- STOP RECORDING ---
                self.stop_recording()
                response.success = True
                response.message = "RECORDING INTERRUPTED & STOPPED."

            self.get_logger().info(response.message)
            return response

def main():
    rclpy.init()
    node = RecordDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_recording()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()