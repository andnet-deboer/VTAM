import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import time

class SimpleHeadCamera(Node):
    def __init__(self):
        super().__init__('simple_head_camera')
        
        # 1. Setup Publishers
        self.image_pub = self.create_publisher(Image, '/head_camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/head_camera/color/camera_info', 10)
        self.bridge = CvBridge()

        # 2. Setup RealSense Pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Head camera usually has a specific serial, but if it's the only D435 connected:
        # config.enable_device('THE_SERIAL_NUMBER') 
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        try:
            self.pipeline.start(config)
            self.get_logger().info("Head Camera Started (640x480 @ 30fps)")
        except RuntimeError as e:
            self.get_logger().error(f"Could not start camera: {e}")
            self.get_logger().error("Try running: stretch_free_robot_process.py")
            raise e

        # 3. Create Timer (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Get Frame
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                return

            # Convert to Numpy
            image_np = np.asanyarray(color_frame.get_data())

            # Create ROS Image Message
            msg = self.bridge.cv2_to_imgmsg(image_np, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"

            # Publish
            self.image_pub.publish(msg)
            
            # (Optional) Publish empty CameraInfo just to keep Rviz happy
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.height = 480
            info_msg.width = 640
            self.info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().warn(f"Frame error: {e}")

    def stop(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleHeadCamera()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
