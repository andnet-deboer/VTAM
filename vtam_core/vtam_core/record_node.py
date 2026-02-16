#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal
from datetime import datetime
import time
import threading
from nav_msgs.msg import Path
import sys
from queue import Queue

class RecordDemoNode(Node):
    def __init__(self):
        super().__init__('record_demo_node')
        self.declare_parameter('demo_name', 'demo')

        # Sequential background worker setup
        self.conversion_queue = Queue()
        self.worker_thread = threading.Thread(target=self._queue_worker, daemon=True)
        self.worker_thread.start()
        
        self.recording_process = None
        self.current_bag_path = None
        self.save_dir = os.path.expanduser("~/VTAM/data/raw")
        os.makedirs(self.save_dir, exist_ok=True)

        # Safety Parameters
        self.max_size_gb = 10.0
        self.max_bytes = self.max_size_gb * 1024 * 1024 * 1024
        
        self.watchdog_timer = self.create_timer(1.0, self.storage_watchdog)

        self.path_pub = self.create_publisher(Path, '/umi_trajectory', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'base_link'
        self.path_msg.poses = []

        self.srv = self.create_service(SetBool, 'record_demo', self.record_callback)
        self.get_logger().info(f'Recorder Active (Limit: {self.max_size_gb}GB)')

    def _queue_worker(self):
        """Processes bags sequentially in the background."""
        while rclpy.ok():
            bag_path = self.conversion_queue.get()
            try:
                self.get_logger().info(f"Worker: Starting {os.path.basename(bag_path)}")
                self.convert_to_zarr(bag_path)
            except Exception as e:
                self.get_logger().error(f"Worker error: {e}")
            finally:
                self.conversion_queue.task_done()

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
        """Cleanly stops recording and queues the bag for conversion."""
        if self.recording_process is not None:
            bag_to_convert = self.current_bag_path 
            self.get_logger().info("Shutting down recording process...")

            # Reset trajectory UI
            try:
                self.path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_msg.poses = [] 
                self.path_pub.publish(self.path_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to clear trajectory: {e}")
            
            try:
                pgid = os.getpgid(self.recording_process.pid)
                os.killpg(pgid, signal.SIGINT)
                
                start_wait = time.time()
                while time.time() - start_wait < 1.0:
                    if self.recording_process.poll() is not None:
                        break
                    time.sleep(0.1)
                
                if self.recording_process.poll() is None:
                    os.killpg(pgid, signal.SIGKILL)

                # Push to background worker
                self.conversion_queue.put(bag_to_convert)

            except Exception as e:
                self.get_logger().error(f"Error during interrupt: {e}")
            finally:
                self.recording_process = None
                self.current_bag_path = None
                self.get_logger().info("System State Reset. Ready for next demo.")

    def storage_watchdog(self):
        if self.recording_process and self.current_bag_path:
            current_size = self.get_dir_size(self.current_bag_path)
            if current_size > self.max_bytes:
                self.get_logger().error("DISK LIMIT EXCEEDED. TRIGGERING STOP.")
                self.stop_recording()

    def record_callback(self, request, response):
        is_running = (self.recording_process is not None and 
                    self.recording_process.poll() is None)

        if not is_running:
            if self.recording_process is not None:
                self.recording_process = None

            # Prepare paths
            name_prefix = self.get_parameter('demo_name').get_parameter_value().string_value
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            task_dir = os.path.join(self.save_dir, name_prefix)
            os.makedirs(task_dir, exist_ok=True)
            self.current_bag_path = os.path.join(task_dir, f"{name_prefix}_{timestamp}")

            topics = [
                "/sync_pulse","/joint_states", "/tf", "/tf_static", "/umi_gripper", "/umi_trajectory",
                "/camera/color/image_raw/compressed", "/camera/color/camera_info",
                "/camera_arm/color/image_rect_raw/compressed", "/camera_arm/color/camera_info",
                 "/tactile_gripper_controller"
            ]

            cmd = [
                "ros2", "bag", "record", "-s", "mcap",
                "-o", self.current_bag_path,
            ] + topics
            
            try:
                self.recording_process = subprocess.Popen(cmd, start_new_session=True)
                
                # Mandatory warm-up delay for buffer initialization
                self.get_logger().info("Initializing MCAP buffers (2s warm-up)...")
                time.sleep(2.0)
                
                response.success = True
                response.message = f"RECORDING LIVE: {self.current_bag_path}"
            except Exception as e:
                response.success = False
                response.message = f"Failed to start: {str(e)}"
        else:
            self.stop_recording()
            response.success = True
            response.message = "RECORDING STOPPED."

        self.get_logger().info(response.message)
        return response
    
    def convert_to_zarr(self, bag_path):
        """Polls for MCAP and runs the conversion script."""
        bag_name = os.path.basename(os.path.normpath(bag_path))
        target_mcap = os.path.join(bag_path, f"{bag_name}_0.mcap")
        
        max_retries = 10
        found = False
        
        self.get_logger().info(f"Waiting for raw MCAP: {bag_name}...")

        for i in range(max_retries):
            if os.path.exists(target_mcap):
                found = True
                break
            time.sleep(0.5)

        if not found:
            self.get_logger().error(f"MCAP timeout: {target_mcap} not found.")
            return
        
        last_size = -1
        while True:
            current_size = os.path.getsize(target_mcap)
            if current_size == last_size and current_size > 0:
                break
            last_size = current_size
            time.sleep(0.5)

        try:
            search_path = os.path.dirname(os.path.abspath(__file__))
            vtam_root = None
            for _ in range(5):
                if os.path.exists(os.path.join(search_path, 'training', 'process_demo.py')):
                    vtam_root = search_path
                    break
                search_path = os.path.dirname(search_path)

            if not vtam_root:
                self.get_logger().error("Could not locate VTAM root.")
                return

            script_path = os.path.join(vtam_root, "training", "process_demo.py")
            python_exe = os.path.join(vtam_root, ".venv", "bin", "python3")

            self.get_logger().info(f"Launching Zarr Conversion: {os.path.basename(target_mcap)}")
            
            # Use 'nice' to prevent conversion from lagging the next recording session
            subprocess.run(
                ["nice", "-n", "15", python_exe, script_path, "--mcap", target_mcap],
                env=os.environ.copy(),
                capture_output=True, 
                text=True, 
                check=True
            )
            self.get_logger().info(f"Zarr Generation Complete for {bag_name}")

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Synchronizer Error: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Unexpected Error: {e}")

def main():
    rclpy.init()
    node = RecordDemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_recording()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()