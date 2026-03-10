#!/usr/bin/env python3
"""
umi_detector_node.py — Live UMI pose detection.
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import os
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo, CompressedImage
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped, PointStamped
from scipy.spatial.transform import Rotation as R_scipy
from ament_index_python.packages import get_package_share_directory
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
        te = 1.0 / self.freq
        return 1.0 / (1.0 + 1.0 / (2 * np.pi * cutoff * te))

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x
        dx = (x - self.x_prev) * self.freq
        edx = self.dx_prev + self._alpha(self.d_cutoff) * (dx - self.dx_prev)
        cutoff = self.min_cutoff + self.beta * np.abs(edx)
        x_filtered = self.x_prev + self._alpha(cutoff) * (x - self.x_prev)
        self.x_prev = x_filtered
        self.dx_prev = edx
        return x_filtered


class AprilTagMarker:
    def __init__(self, tag_id, marker_info):
        self.info = marker_info.get(str(tag_id), {'length_mm': 64.0, 'frames': {}})
        self.length_mm = self.info.get('length_mm', 64.0)
        self.pos = None
        self.axes = [None, None, None]

    def update(self, corners, K, D):
        half = self.length_mm / 2.0
        pts = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        _, rvec, tvec = cv2.solvePnP(pts, corners, K, D)
        self.pos = tvec.flatten() / 1000.0
        R = cv2.Rodrigues(rvec)[0]
        self.axes = [-R[:3, 1], -R[:3, 0], -R[:3, 2]]


def cube_pose_from_tag(marker, config):
    t = config.get('trans', [0, 0, 0])
    pos = marker.pos + t[0]*marker.axes[0] + t[1]*marker.axes[1] + t[2]*marker.axes[2]
    R_tag = np.column_stack(marker.axes)
    R_offset = R_scipy.from_quat(config.get('quat', [0, 0, 0, 1])).as_matrix()
    R_cube = R_tag @ R_offset.T
    return pos, R_scipy.from_matrix(R_cube).as_quat()


def tag_weight(corners):
    return np.clip(cv2.contourArea(corners.reshape(4, 2)) / 1000.0, 0.1, 1.0)


def average_quaternions(quats, weights):
    if not quats: return np.array([0, 0, 0, 1])
    quats = [np.array(q) / np.linalg.norm(q) for q in quats]
    aligned = [q if np.dot(quats[0], q) >= 0 else -q for q in quats]
    Q = sum(w * np.outer(q, q) for q, w in zip(aligned, weights))
    return np.linalg.eigh(Q)[1][:, -1]


class UmiDetectorNode(Node):
    def __init__(self):
        super().__init__('umi_detector_node')

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        yaml_path = os.path.join(
            get_package_share_directory('vtam_core'),
            'config', 'teleop_april_marker_info_86mm.yaml'
        )
        try:
            with open(yaml_path, 'r') as f:
                self.marker_info = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Could not load marker YAML: {e}")
            self.marker_info = {}

        self.declare_parameter('pos_min_cutoff', 0.1)
        self.declare_parameter('pos_beta', 3.0)
        self.declare_parameter('quat_min_cutoff', 0.1)
        self.declare_parameter('quat_beta', 3.0)
        self.add_on_set_parameters_callback(self._param_cb)

        self.pos_filter  = OneEuroFilter(30.0, self.get_parameter('pos_min_cutoff').value,  self.get_parameter('pos_beta').value)
        self.quat_filter = OneEuroFilter(30.0, self.get_parameter('quat_min_cutoff').value, self.get_parameter('quat_beta').value)

        params = aruco.DetectorParameters()
        params.cornerRefinementMethod  = aruco.CORNER_REFINE_SUBPIX
        params.adaptiveThreshWinSizeMin  = 3
        params.adaptiveThreshWinSizeMax  = 23
        params.adaptiveThreshWinSizeStep = 10
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11), params
        )

        self.pose_pub     = self.create_publisher(PoseStamped,  'umi_cube_pose',           10)
        self.gripper_pub  = self.create_publisher(PoseStamped,  'umi_gripper_pose',         10)
        self.raw_pose_pub = self.create_publisher(PoseStamped,  'umi_cube_pose_raw',        10)
        self.centroid_pub = self.create_publisher(PointStamped, '/umi_cube/pixel_centroid', 10)

        self.create_subscription(CameraInfo,      '/camera/color/camera_info',         self._info_cb,  10)
        self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self._image_cb,  1)
        self.create_service(Trigger, '/umi_detector/shutdown', self._shutdown_cb)

        self.camera_info = None
        self.collection  = {}
        self.prev_quat   = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _param_cb(self, params):
        for p in params:
            if p.name == 'pos_min_cutoff':  self.pos_filter.update_params(min_cutoff=p.value)
            if p.name == 'pos_beta':        self.pos_filter.update_params(beta=p.value)
            if p.name == 'quat_min_cutoff': self.quat_filter.update_params(min_cutoff=p.value)
            if p.name == 'quat_beta':       self.quat_filter.update_params(beta=p.value)
        return SetParametersResult(successful=True)

    def _info_cb(self, msg):
        self.camera_info = {
            'K': np.array(msg.k).reshape(3, 3),
            'D': np.array(msg.d)
        }

    def _image_cb(self, msg):
        if self.camera_info is None:
            return

        # Non-blocking TF lookup — drop frame if not available yet
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                rclpy.time.Time()  # latest available, zero wait
            )
        except Exception:
            return

        img   = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        gray  = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return

        K, D = self.camera_info['K'], self.camera_info['D']
        candidates = []
        cube_corners = []

        for c, aid in zip(corners, ids.flatten()):
            aid = int(aid)
            if aid not in self.collection:
                self.collection[aid] = AprilTagMarker(aid, self.marker_info)
            m = self.collection[aid]
            m.update(c[0], K, D)
            frames = m.info.get('frames', {})
            if 'umi_cube' in frames:
                pos, quat = cube_pose_from_tag(m, frames['umi_cube'])
                candidates.append((pos, quat, tag_weight(c[0])))
                cube_corners.append(c[0])

        if not candidates:
            return

        # Weighted fusion in camera frame
        w = np.array([c[2] for c in candidates])
        w /= w.sum()
        pos_cam  = np.average([c[0] for c in candidates], axis=0, weights=w)
        quat_cam = average_quaternions([c[1] for c in candidates], w)

        # Camera → base_link
        ro = tf.transform.rotation
        tr = tf.transform.translation
        R_bc = R_scipy.from_quat([ro.x, ro.y, ro.z, ro.w])
        T_bc = np.array([tr.x, tr.y, tr.z])

        fused_pos  = T_bc + R_bc.apply(pos_cam)
        live_pitch = R_bc.as_euler('xyz', degrees=True)[1]
        R_fix      = R_scipy.from_euler('zyx', [0, -live_pitch, 180], degrees=True)
        fused_quat = (R_bc * R_scipy.from_quat(quat_cam) * R_fix).as_quat()

        stamp = msg.header.stamp

        # Publish raw
        self._publish_pose(self.raw_pose_pub, stamp, 'base_link', fused_pos, fused_quat)

        # Filter
        fused_pos = self.pos_filter.filter(fused_pos)
        if self.prev_quat is not None and np.dot(fused_quat, self.prev_quat) < 0:
            fused_quat = -fused_quat
        fused_quat = self.quat_filter.filter(fused_quat)
        fused_quat /= np.linalg.norm(fused_quat)
        self.prev_quat = fused_quat.copy()

        # Z 90° correction
        quat_corrected = (R_scipy.from_quat(fused_quat) * R_scipy.from_euler('z', 90, degrees=True)).as_quat()

        # Broadcast TF frames
        self._broadcast_tf('umi_disconnect', 'base_link',     fused_pos,              quat_corrected, stamp)
        self._broadcast_tf('fiducial_cube',  'umi_disconnect', [-0.0575, 0.0, 0.061], [0,0,0,1],      stamp)
        self._broadcast_tf('umi_gripper',    'umi_disconnect', [0.242,   0.0, 0.0],   [0,0,0,1],      stamp)

        # Publish pose topics
        self._publish_pose(self.pose_pub,    stamp, 'base_link',     fused_pos, fused_quat)
        self._publish_pose(self.gripper_pub, stamp, 'umi_disconnect', [0.242, 0.0, 0.0], [0,0,0,1])

        # Pixel centroid
        if cube_corners:
            cx = np.mean([c[:, 0].mean() for c in cube_corners])
            cy = np.mean([c[:, 1].mean() for c in cube_corners])
            pt = PointStamped()
            pt.header.stamp    = stamp
            pt.header.frame_id = 'camera_color_optical_frame'
            pt.point.x = float(cx)
            pt.point.y = float(cy)
            self.centroid_pub.publish(pt)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_pose(self, pub, stamp, frame_id, pos, quat):
        msg = PoseStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quat)
        pub.publish(msg)

    def _broadcast_tf(self, child, parent, pos, quat, stamp):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, quat)
        self.tf_broadcaster.sendTransform(t)

    def _shutdown_cb(self, request, response):
        response.success = True
        response.message = "Shutting down"
        self.create_timer(0.1, lambda: rclpy.shutdown())
        return response


def main():
    rclpy.init()
    node = UmiDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()