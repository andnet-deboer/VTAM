import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import os
import signal
from datetime import datetime

class RecordDemoNode(Node):
    def __init__(self):
        super().__init__('record_demo_node')
        self.declare_parameter('demo_name', 'demo')
        self.recording_process = None
        self.save_dir = os.path.expanduser("~/VTAM/data/raw")
        os.makedirs(self.save_dir, exist_ok=True)

        self.srv = self.create_service(SetBool, 'record_demo', self.record_callback)
        self.get_logger().info('✓ Targeted VTAM Recorder Ready')

    def record_callback(self, request, response):
        if self.recording_process is None or self.recording_process.poll() is not None:
            # --- START RECORDING ---
            name_prefix = self.get_parameter('demo_name').get_parameter_value().string_value
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            full_path = os.path.join(self.save_dir, f"{name_prefix}_{timestamp}")

            # Define the minimal viable topic set for VTAM
            topics = [
                "/camera/head_camera/color/image_raw",     # For SAM 3 / FoundationPose
                "/camera/head_camera/color/camera_info",   # For 3D projection
                "/joint_states",                          # Proprioception
                "/tf",                                    # Coordinate transforms
                "/tf_static",                             # Static robot geometry
                "/any_skin/data"                          # Tactile feedback
            ]

            # Construct the command with the explicit topic list
            cmd = f"ros2 bag record -s mcap -o {full_path} {' '.join(topics)}"
            
            try:
                self.recording_process = subprocess.Popen(
                    cmd, 
                    shell=True, 
                    executable='/bin/bash', 
                    start_new_session=True
                )
                response.success = True
                response.message = f"RECORDING STARTED: {full_path}"
            except Exception as e:
                response.success = False
                response.message = f"Failed to start: {str(e)}"
        else:
            # --- Stop Recording
            # Send SIGINT so MCAP writes its metadata/index footer
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            self.recording_process = None
            
            response.success = True
            response.message = "RECORDING STOPPED & SAVED."

        self.get_logger().info(response.message)
        return response

def main():
    rclpy.init()
    node = RecordDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()