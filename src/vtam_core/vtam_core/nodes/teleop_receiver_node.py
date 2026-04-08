#!/usr/bin/env python3
"""
teleop_receiver_node.py — TeleTool2 Socket.IO → ROS 2 PoseStamped bridge.

Receives ARKit 6D pose from TeleTool2 iOS app over Socket.IO (WebSocket),
transforms from ARKit frame to ROS frame, and publishes as PoseStamped.

TeleTool2 protocol:
    Event:  'update'
    Data:   {"transformMatrix": [[4x4 SE3]], "timestamp": int, "volumeButtonState": ""}

ARKit → ROS frame transform:
    ARKit: X=right, Y=up,      Z=toward-viewer  (right-handed, Y-up)
    ROS:   X=forward, Y=left,  Z=up             (right-handed, Z-up)
    Translation: ros_x = -arkit_z
                 ros_y = -arkit_x
                 ros_z =  arkit_y
    Rotation:    apply the same axis-swap as a change-of-basis

Published Topics:
    /phone_teleop/pose  (geometry_msgs/PoseStamped)  — phone pose in ROS frame

Usage:
    ros2 run vtam_core teleop_receiver_node
    ros2 run vtam_core teleop_receiver_node --ros-args -p port:=8080
"""

import asyncio
import queue
import threading
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import socketio
from aiohttp import web


# ARKit-to-ROS axis-swap rotation matrix
# Columns are: [what ARKit-X becomes, ARKit-Y becomes, ARKit-Z becomes] in ROS
# ros_x = -arkit_z  →  ARKit Z column maps to -ROS X
# ros_y = -arkit_x  →  ARKit X column maps to -ROS Y
# ros_z =  arkit_y  →  ARKit Y column maps to  ROS Z
_R_ARKIT_TO_ROS = np.array([
    [ 0, 0, -1],   # ROS X = -ARKit Z
    [-1, 0,  0],   # ROS Y = -ARKit X
    [ 0, 1,  0],   # ROS Z =  ARKit Y
], dtype=float)


def arkit_matrix_to_ros_pose(matrix_4x4):
    """
    Convert a 4x4 ARKit SE(3) transform matrix to ROS position + quaternion.

    The ARKit matrix is column-major in the API but arrives as a list of rows:
        [[r00, r01, r02, tx],
         [r10, r11, r12, ty],
         [r20, r21, r22, tz],
         [  0,   0,   0,  1]]

    Returns:
        pos: (3,) float array  [x, y, z] in ROS frame
        quat: (4,) float array [x, y, z, w] in ROS frame
    """
    M = np.array(matrix_4x4, dtype=float)

    # Extract translation (last column, first 3 rows) in ARKit frame
    arkit_t = M[:3, 3]

    # Apply axis swap to translation
    ros_t = _R_ARKIT_TO_ROS @ arkit_t

    # Extract rotation (upper-left 3x3) in ARKit frame
    arkit_R = M[:3, :3]

    # Change of basis: R_ros = R_arkit_to_ros @ R_arkit @ R_arkit_to_ros^T
    ros_R = _R_ARKIT_TO_ROS @ arkit_R @ _R_ARKIT_TO_ROS.T

    quat_xyzw = Rotation.from_matrix(ros_R).as_quat()

    return ros_t, quat_xyzw


class TeleopReceiverNode(Node):
    def __init__(self):
        super().__init__('teleop_receiver_node')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        self._pub = self.create_publisher(PoseStamped, '/phone_teleop/pose', 10)

        # Thread-safe queue: socket thread → ROS publish timer
        self._pose_queue = queue.Queue(maxsize=5)

        # Publish timer at 60 Hz (drains queue, publishes latest)
        self.create_timer(1.0 / 60.0, self._publish_latest)

        # Start Socket.IO server in background thread
        self._server_thread = threading.Thread(
            target=self._run_socketio_server,
            args=(port,),
            daemon=True,
        )
        self._server_thread.start()

        self.get_logger().info(
            f'TeleTool2 receiver started. '
            f'Connect your iPhone to this machine on port {port}. '
            f'Publishing on /phone_teleop/pose'
        )

    # ── Socket.IO server (runs in its own thread + event loop) ────────────────

    def _run_socketio_server(self, port):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        sio = socketio.AsyncServer(cors_allowed_origins='*', async_mode='aiohttp')
        app = web.Application()
        sio.attach(app)

        @sio.event
        async def connect(sid, environ):
            self.get_logger().info(f'TeleTool2 connected (sid={sid})')

        @sio.on('update')
        async def on_update(sid, data):
            try:
                if isinstance(data, str):
                    import json
                    data = json.loads(data)
                matrix = data['transformMatrix']
                pos, quat = arkit_matrix_to_ros_pose(matrix)
                # Drop oldest if queue is full (prefer fresh data)
                if self._pose_queue.full():
                    try:
                        self._pose_queue.get_nowait()
                    except queue.Empty:
                        pass
                self._pose_queue.put_nowait((pos, quat))
            except Exception as e:
                self.get_logger().warn(f'Failed to parse update: {e}')

        @sio.event
        async def disconnect(sid):
            self.get_logger().info(f'TeleTool2 disconnected (sid={sid})')

        loop.run_until_complete(
            web._run_app(app, host='0.0.0.0', port=port)
        )

    # ── ROS publish timer ─────────────────────────────────────────────────────

    def _publish_latest(self):
        if self._pose_queue.empty():
            return

        # Drain queue, publish only the latest
        pos, quat = None, None
        while not self._pose_queue.empty():
            try:
                pos, quat = self._pose_queue.get_nowait()
            except queue.Empty:
                break

        if pos is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()