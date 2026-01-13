import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import cv2
import json
import os
import time
from datetime import datetime

# Import utilities from the sibling 'stretch_ai' package if available
# or use standard ROS 2 CV Bridge
from cv_bridge import CvBridge

class UniversalRecorder(Node):
    def __init__(self):
        super().__init__('vtam_recorder')
        self.bridge = CvBridge()
        
        # State buffers
        self.latest_tactile = []
        self.latest_joints = {}
        self.recording = False
        self.save_path = ""
        
        # Subscribers
        self.create_subscription(Float32MultiArray, '/tactile_sensors', self.tactile_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        # Add Camera Subscriber here (e.g. /camera/color/image_raw)
        
    def tactile_cb(self, msg):
        self.latest_tactile = msg.data

    def joint_cb(self, msg):
        # Map joint names to positions
        self.latest_joints = dict(zip(msg.name, msg.position))

    def start_recording(self, task_name):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_path = os.path.join(os.path.expanduser('~/vtam_data'), f"{task_name}_{timestamp}")
        os.makedirs(self.save_path, exist_ok=True)
        self.recording = True
        self.get_logger().info(f"STARTED recording: {task_name}")
        
        # Open data files (JSON lines for simple streaming)
        self.log_file = open(os.path.join(self.save_path, "data.jsonl"), "w")

    def stop_recording(self):
        self.recording = False
        if hasattr(self, 'log_file'):
            self.log_file.close()
        self.get_logger().info("STOPPED recording.")

    def step(self):
        # Called by the main loop to save a frame
        if self.recording:
            entry = {
                "timestamp": time.time(),
                "tactile": list(self.latest_tactile),
                "joints": self.latest_joints
            }
            self.log_file.write(json.dumps(entry) + "\n")

def main(args=None):
    # This node is usually run by the Teach Mode manager, not standalone
    pass