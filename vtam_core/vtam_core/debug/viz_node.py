#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class TactileVizNode(Node):
    def __init__(self):
        super().__init__('tactile_viz')

        # --- Parameters ---
        self.declare_parameter('max_pressure', 400.0) # Adjust sensitivity here
        self.max_pressure = self.get_parameter('max_pressure').value

        self.bridge = CvBridge()

        # --- Subscribers ---
        # Left
        self.sub_left = self.create_subscription(
            Float32MultiArray, '/tactile_left', self.left_callback, 10)
        # Right (New)
        self.sub_right = self.create_subscription(
            Float32MultiArray, '/tactile_right', self.right_callback, 10)

        # UMI Gripper
        self.sub_umi = self.create_subscription(
            Float32MultiArray, '/tactile_gripper_controller', self.umi_callback, 10)

        # --- Publishers (Output Images) ---
        # Left
        self.pub_left_img = self.create_publisher(Image, '/viz/tactile_left', 10)
        # Right (New)
        self.pub_right_img = self.create_publisher(Image, '/viz/tactile_right', 10)
        
        # UMI Gripper (New)
        self.pub_umi_img = self.create_publisher(Image, '/viz/tactile_umi', 10)
        # Chip locations (approximate pixel coordinates for 300x300 image)
        # [x, y]

        self.locations = np.array([
            [150, 150], # Center
            [90, 150],  # Left
            [210, 150], # Right
            [150, 90],  # Top
            [150, 210]  # Bottom
        ])

    def draw_heatmap(self, data_flat):
        # Create a black background (300x300)
        img = np.zeros((300, 300, 3), dtype=np.uint8)
        
        # Reshape to (5 sensors, 3 axes)
        if len(data_flat) != 15: return img
        data = np.array(data_flat).reshape(-1, 3)
        
        # Calculate Magnitude (Total Pressure) for each of the 5 chips
        mags = np.linalg.norm(data, axis=1)

        for i, mag in enumerate(mags):
            # Normalize pressure (0 to 1)
            intensity = min(mag / self.max_pressure, 1.0)
            
            # Color Mapping: Blue (low) -> Red (high)
            # OpenCV uses BGR format
            red = int(255 * intensity)
            blue = int(255 * (1 - intensity))
            green = 0
            color = (blue, green, red)
            
            # Radius grows with pressure
            radius = int(20 + (intensity * 30))
            
            # Draw filled circle
            center = (self.locations[i][0], self.locations[i][1])
            cv2.circle(img, center, radius, color, -1)
            
            # Draw white outline
            cv2.circle(img, center, radius, (255, 255, 255), 2)

        return img

    def left_callback(self, msg):
        # 1. Generate Image
        cv_image = self.draw_heatmap(msg.data)
        # 2. Convert to ROS Msg
        ros_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        # 3. Publish
        self.pub_left_img.publish(ros_msg)

    def right_callback(self, msg):
        # 1. Generate Image
        cv_image = self.draw_heatmap(msg.data)
        # 2. Convert to ROS Msg
        ros_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        # 3. Publish
        self.pub_right_img.publish(ros_msg)

    def umi_callback(self, msg):
        # 1. Generate Image
        cv_image = self.draw_heatmap(msg.data)
        # 2. Convert to ROS Msg
        ros_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        # 3. Publish
        self.pub_umi_img.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TactileVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()