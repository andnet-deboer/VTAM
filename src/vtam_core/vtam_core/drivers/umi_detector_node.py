#!/usr/bin/env python3
"""
UMI Cube Tracker - BASE_LINK ALIGNED VERSION
"""

import cv2
import numpy as np
import cv2.aruco as aruco 
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from scipy.spatial.transform import Rotation as R_scipy
from ament_index_python.packages import get_package_share_directory
import os
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger

class OneEuroFilter:
    def __init__(self, freq, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = None
        
    def update_params(self, min_cutoff=None, beta=None):
        if min_cutoff is not None: self.min_cutoff = min_cutoff
        if beta is not None: self.beta = beta

    def _alpha(self, cutoff):
        tau = 1.0 / (2 * np.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x
        dx = (x - self.x_prev) * self.freq
        edx = self.dx_prev + self._alpha(self.d_cutoff) * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        alpha = self._alpha(cutoff)
        x_filtered = self.x_prev + alpha * (x - self.x_prev)
        self.x_prev = x_filtered
        self.dx_prev = edx
        return x_filtered

def get_cube_pose_from_tag(tag_pos, tag_x, tag_y, tag_z, trans_offset, quat_tag_to_cube):
    cube_pos = tag_pos + (trans_offset[0] * tag_x) + (trans_offset[1] * tag_y) + (trans_offset[2] * tag_z)
    R_tag_in_world = np.column_stack((tag_x, tag_y, tag_z))
    R_tag_to_cube = R_scipy.from_quat(quat_tag_to_cube).as_matrix()
    R_cube_in_world = R_tag_in_world @ R_tag_to_cube.T
    return {'pos': cube_pos, 'quat': R_scipy.from_matrix(R_cube_in_world).as_quat()}

def calculate_tag_weight(corners):
    pts = corners.reshape(4, 2)
    area = cv2.contourArea(pts)
    return np.clip(area / 1000.0, 0.1, 1.0)

def average_quaternions(quaternions, weights):
    if len(quaternions) == 0: return np.array([0, 0, 0, 1])
    quaternions = [np.array(q) / np.linalg.norm(q) for q in quaternions]
    q_ref = quaternions[0]
    aligned_quats = []
    for q in quaternions:
        dot = np.dot(q_ref, q)
        aligned_quats.append(q if dot >= 0 else -q)
    Q = np.zeros((4, 4))
    for q, w in zip(aligned_quats, weights):
        Q += w * np.outer(q, q)
    return np.linalg.eigh(Q)[1][:, -1]

class AprilTagMarker:
    def __init__(self, tag_id, marker_info):
        self.tag_id = tag_id
        self.info = marker_info.get(str(tag_id), {'length_mm': 64.0, 'frames': {}})
        self.length_mm = self.info.get('length_mm', 64.0)
        self.pos = None
        self.axes = [None, None, None]
    
    def update(self, corners, camera_matrix, dist_coeffs):
        half = self.length_mm / 2.0
        points_3D = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        _, rvec, tvec = cv2.solvePnP(points_3D, corners, camera_matrix, dist_coeffs)
        self.pos = tvec.flatten() / 1000.0
        R_mat = cv2.Rodrigues(rvec)[0]
        self.axes = [-R_mat[:3, 1], -R_mat[:3, 0], -R_mat[:3, 2]]

class UmiDetectorNode(Node):
    def __init__(self):
        super().__init__('umi_detector_node')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, '/umi_detector/shutdown', self._shutdown_cb)

        package_share_directory = get_package_share_directory('vtam_core')
        yaml_path = os.path.join(package_share_directory, 'config', 'teleop_april_marker_info_86mm.yaml')
        
        # Load it once, properly
        try:
            with open(yaml_path, 'r') as f: 
                self.marker_info = yaml.safe_load(f)
                self.get_logger().info(f"Successfully loaded marker info from: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Could not load YAML from {yaml_path}. Error: {e}")
            self.marker_info = {}


        self.pose_pub = self.create_publisher(PoseStamped, 'umi_cube_pose', 10)
        self.gripper_pub = self.create_publisher(PoseStamped, 'umi_gripper_pose', 10)
        self.centroid_pub = self.create_publisher(PointStamped, '/umi_cube/pixel_centroid', 10)
        self.raw_pose_pub = self.create_publisher(PoseStamped, 'umi_cube_pose_raw', 10)

        self.declare_parameter('pos_min_cutoff', 0.1)
        self.declare_parameter('pos_beta', 3.0)
        self.declare_parameter('quat_min_cutoff', 0.1)
        self.declare_parameter('quat_beta', 3.0)
        self.declare_parameter('off_r', 180.0)
        self.declare_parameter('off_p', 0.0)
        self.declare_parameter('off_y', 0.0)

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())

        self.pos_filter = OneEuroFilter(30.0, self.get_parameter('pos_min_cutoff').value, self.get_parameter('pos_beta').value)
        self.quat_filter = OneEuroFilter(30.0, self.get_parameter('quat_min_cutoff').value, self.get_parameter('quat_beta').value)
        self.add_on_set_parameters_callback(self.param_cb)

        # TODO
        # self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        # self.detector = aruco.ArucoDetector(self.dictionary, aruco.DetectorParameters())

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        
        # Configure detector parameters for better AprilTag accuracy on compressed images
        params = aruco.DetectorParameters()
        
        # Turn on Subpixel Refinement to defeat JPEG artifacts and edge blur
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        
        # (Optional) Make the thresholding more forgiving for lighting changes
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        
        # Pass the custom params to the detector
        self.detector = aruco.ArucoDetector(self.dictionary, params)
        ## End TODO

        self.collection, self.camera_info_dict, self.prev_cube_pose = {}, None, None
        
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(
            CompressedImage, 
            '/camera/color/image_raw/compressed', 
            self.image_cb,
            1
        )

    def param_cb(self, params):
        for p in params:
            if p.name == 'pos_min_cutoff': self.pos_filter.update_params(min_cutoff=p.value)
            if p.name == 'pos_beta': self.pos_filter.update_params(beta=p.value)
            if p.name == 'quat_min_cutoff': self.quat_filter.update_params(min_cutoff=p.value)
            if p.name == 'quat_beta': self.quat_filter.update_params(beta=p.value)
        return SetParametersResult(successful=True)

    def info_cb(self, msg):
        self.camera_info_dict = {'camera_matrix': np.array(msg.k).reshape((3, 3)), 'distortion_coefficients': np.array(msg.d)}

    def _recording_cb(self, msg):
        if msg.data:  # True = new episode starting
            self.trajectory.poses = []
            self.get_logger().info("Trajectory reset for new episode.")

    def image_cb(self, msg):
        if self.camera_info_dict is None: return
        # Decompressing JPEG is much faster than moving raw 720p buffers
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None: return
        
        cube_candidates = []
        for c, aid in zip(corners, ids.flatten()):
            aid = int(aid)
            if aid not in self.collection: self.collection[aid] = AprilTagMarker(aid, self.marker_info)
            marker = self.collection[aid]
            marker.update(c[0], self.camera_info_dict['camera_matrix'], self.camera_info_dict['distortion_coefficients'])
            frames = marker.info.get('frames', {})
            if 'umi_cube' in frames:
                config = frames['umi_cube']
                res = get_cube_pose_from_tag(marker.pos, marker.axes[0], marker.axes[1], marker.axes[2], 
                                            config.get('trans', [0.0,0.0,0.0]), config.get('quat', [0.0,0.0,0.0,1.0]))
                cube_candidates.append({'pos': res['pos'], 'quat': res['quat'], 'weight': calculate_tag_weight(c[0])})
        
        if cube_candidates:
            # Detection relative to Camera (solvePnP output)
            weights = np.array([c['weight'] for c in cube_candidates])
            f_pos_cam = np.average([c['pos'] for c in cube_candidates], axis=0, weights=weights/sum(weights))
            f_quat_cam = average_quaternions([c['quat'] for c in cube_candidates], weights/sum(weights))
            
            try:
                # Use rclpy.time.Time() to grab the absolute latest data available
                try:
                    t = self.tf_buffer.lookup_transform(
                        'base_link', 
                        'camera_color_optical_frame', 
                        msg.header.stamp,
                        # Wait up to 100ms for the TF tree to populate
                        timeout=rclpy.duration.Duration(seconds=0.1) 
                    )
                except Exception as e:
                    # Just return and wait for the next frame
                    return
                
                # Position: Places the cube in the world based on the latest camera pose
                R_base_cam = R_scipy.from_quat([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                T_base_cam = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                fused_pos = T_base_cam + R_base_cam.apply(f_pos_cam)
                
                # Orientation: Uses the latest head tilt to "un-sway" the cube
                cam_rpy = R_base_cam.as_euler('xyz', degrees=True)
                live_pitch = cam_rpy[1] 
                R_cube_world = R_base_cam * R_scipy.from_quat(f_quat_cam)
                R_fix = R_scipy.from_euler('zyx', [0, -live_pitch, 180], degrees=True)
                fused_quat = (R_cube_world * R_fix).as_quat()

            except Exception as e:
                self.get_logger().warn(f"TF Lookup failed: {e}")
                return
            
            # Timestamp for raw and filtered poses
            stamp = msg.header.stamp

            raw_msg = PoseStamped()
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = 'base_link'
            raw_msg.pose.position.x, raw_msg.pose.position.y, raw_msg.pose.position.z = map(float, fused_pos)
            raw_msg.pose.orientation.x, raw_msg.pose.orientation.y, raw_msg.pose.orientation.z, raw_msg.pose.orientation.w = map(float, fused_quat)
            self.raw_pose_pub.publish(raw_msg)
                
            # Filtering
            fused_pos = self.pos_filter.filter(fused_pos)
            if self.prev_cube_pose is not None and np.dot(fused_quat, self.prev_cube_pose['quat']) < 0:
                fused_quat = -fused_quat
            fq_raw = self.quat_filter.filter(fused_quat)
            fused_quat = fq_raw / np.linalg.norm(fq_raw)

            # Define corrective rotation (e.g., 90 deg around X)
            R_correction = R_scipy.from_euler('z', 90, degrees=True)

            # Apply it to the fused orientation
            R_disconnect_raw = R_scipy.from_quat(fused_quat)

            fused_quat_corrected = (R_disconnect_raw * R_correction).as_quat()

            self.prev_cube_pose = {'pos': fused_pos, 'quat': fused_quat}

            all_corners = [c[0] for c, aid in zip(corners, ids.flatten()) 
               if int(aid) in self.collection and 
               'umi_cube' in self.collection[int(aid)].info.get('frames', {})]
            if all_corners:
                cx = np.mean([c[:, 0].mean() for c in all_corners])
                cy = np.mean([c[:, 1].mean() for c in all_corners])
                pt = PointStamped()
                pt.header.stamp = stamp
                pt.header.frame_id = 'camera_color_optical_frame'
                pt.point.x = float(cx)
                pt.point.y = float(cy)
                self.centroid_pub.publish(pt)
            # Broadcast the main filtered handle pose (now called umi_disconnect)
            # Parent: base_link
            self.broadcast_frame_quat('umi_disconnect', fused_pos, fused_quat_corrected, 'base_link', stamp)
            
            # Project the Fiducial Cube (The actual physical cube location)
            # Offset: 57.5mm back (-X) and 61mm up (+Z) relative to disconnect point
            # Parent: umi_disconnect
            fiducial_offset = [-0.0575, 0.0, 0.061]
            self.broadcast_frame_quat('fiducial_cube', fiducial_offset, [0.0, 0.0, 0.0, 1.0], 'umi_disconnect', stamp)

            # Project the Gripper (The grasp center)
            # Parent: umi_disconnect
            self.broadcast_frame_quat('umi_gripper', [0.242, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 'umi_disconnect', stamp)
            
            # Publish Pose Topic for the disconnect point
            p_msg = PoseStamped()
            p_msg.header.stamp = stamp
            p_msg.header.frame_id = 'base_link'
            p_msg.pose.position.x, p_msg.pose.position.y, p_msg.pose.position.z = map(float, fused_pos)
            p_msg.pose.orientation.x, p_msg.pose.orientation.y, p_msg.pose.orientation.z, p_msg.pose.orientation.w = map(float, fused_quat)
            self.pose_pub.publish(p_msg)

            # Publish Pose Topic for the gripper (relative to handle)
            g_msg = PoseStamped()
            g_msg.header.stamp = p_msg.header.stamp
            g_msg.header.frame_id = 'umi_disconnect'
            g_msg.pose.position.x = 0.242
            g_msg.pose.position.y = 0.0
            g_msg.pose.position.z = 0.0
            g_msg.pose.orientation.w = 1.0 
            self.gripper_pub.publish(g_msg)

    def broadcast_frame_quat(self, name, pos, quat, parent, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, quat)
        self.tf_broadcaster.sendTransform(t)

    def _shutdown_cb(self, request, response):
        response.success = True
        response.message = "Shutting down"
        self.get_logger().info("Shutdown requested, exiting.")
        # Deferred so the response gets sent first
        self.create_timer(0.1, lambda: rclpy.shutdown())
        return response

def main():
    rclpy.init()
    node = UmiDetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()