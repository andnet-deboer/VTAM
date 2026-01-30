import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R

class HighSpeedTracker(Node):
    def __init__(self):
        super().__init__('umi_fast_tracker')
        
        # 1. Subscribe to the EFFICIENT C++ driver (Best Effort is crucial for video)
        self.create_subscription(
            Image, 
            '/head_camera/color/image_raw', 
            self.image_callback, 
            rclpy.qos.qos_profile_sensor_data)
            
        self.create_subscription(
            CameraInfo, 
            '/head_camera/color/camera_info', 
            self.info_callback, 
            10)

        # 2. Publish the LIGHTWEIGHT result (Reliable is fine for small poses)
        self.pose_pub = self.create_publisher(PoseStamped, '/teleop/gripper_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # SETUP YOUR MARKER HERE
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        self.marker_size = 0.05  # 50mm

        self.get_logger().info("High-Speed Tracker Initialized. Waiting for camera...")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None: return

        # 1. Convert (Zero-Copy optimization if possible, but cv_bridge is fast enough locally)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Detect (This happens ON THE ROBOT, extremely fast)
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None and 135 in ids:
            index = np.where(ids == 135)[0][0]
            
            # 3. Solve PnP (Get 3D Pose)
            # Note: For flat markers, solvePnP is more stable than estimatePoseSingleMarkers
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[index], self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            # 4. Publish Result IMMEDIATELY
            self.publish_pose(tvec[0][0], rvec[0][0], msg.header)

    def publish_pose(self, tvec, rvec, header):
        p = PoseStamped()
        p.header = header
        p.pose.position.x = tvec[0]
        p.pose.position.y = tvec[1]
        p.pose.position.z = tvec[2]
        
        # Convert Rodriguez (rvec) to Quaternion
        rot_matrix, _ = cv2.Rodrigues(rvec)
        quat = R.from_matrix(rot_matrix).as_quat() # x, y, z, w
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]

        self.pose_pub.publish(p)

def main():
    rclpy.init()
    node = HighSpeedTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
