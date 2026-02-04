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
        
        # Declare a parameter for the custom name prefix
        self.declare_parameter('demo_name', 'demo')
        
        self.recording_process = None
        self.save_dir = os.path.expanduser("~/VTAM/data/raw")
        
        os.makedirs(self.save_dir, exist_ok=True)

        self.srv = self.create_service(SetBool, 'record_demo', self.record_callback)
        self.get_logger().info('✓ Demo Recorder Service Ready.')

    def record_callback(self, request, response):
        if request.data:
            if self.recording_process is None:
                # 1. Fetch the name from the parameter
                name_prefix = self.get_parameter('demo_name').get_parameter_value().string_value
                
                # 2. Generate the directory name: name_YYYYMMDD_HHMMSS
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                folder_name = f"{name_prefix}_{timestamp}"
                full_path = os.path.join(self.save_dir, folder_name)

                # 3. Start subprocess
                cmd = ['ros2', 'bag', 'record', '-a', '-s', 'mcap', '-o', full_path]
                
                # preexec_fn=os.setsid allows us to kill the whole process group later
                self.recording_process = subprocess.Popen(cmd, preexec_fn=os.setsid)
                
                response.success = True
                response.message = f"Started recording to {full_path}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Already recording!"
        else:
            if self.recording_process:
                # 4. Cleanly stop the recording
                os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
                self.recording_process = None
                
                response.success = True
                response.message = "Stopped recording."
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Not currently recording."
        
        return response

def main():
    rclpy.init()
    node = RecordDemoNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()