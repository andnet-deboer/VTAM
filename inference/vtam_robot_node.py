import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import zmq, pickle, cv2, time, numpy as np
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
    'joint_gripper_finger_right'
]

# Match HelloStretchIdx
ORDERED_JOINTS = [
    'base_x', 'base_y', 'base_theta',
    'joint_lift', 'joint_arm_l0',
    'joint_gripper_finger_right', 'joint_wrist_roll',
    'joint_wrist_pitch', 'joint_wrist_yaw'
]


class VTAMRobotNode(Node):
    def __init__(self):
        super().__init__('vtam_robot_node')

        self.img = None
        self.joints = {}
        self.joint_vels = {}
        self.step = 0
        self.camera_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])

        self.ctx = zmq.Context()
        self.zmq_pubs = {p: self.ctx.socket(zmq.PUB) for p in [4401, 4403, 4404]}
        for port, sock in self.zmq_pubs.items():
            sock.bind(f"tcp://*:{port}")

        self.action_sub = self.ctx.socket(zmq.SUB)
        self.action_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.action_sub.setsockopt(zmq.CONFLATE, 1)
        self.action_sub.bind("tcp://*:4402")

        self.bridge = CvBridge()

        self._cmd_vel = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')

        self.create_subscription(Image, '/gripper_camera/color/image_rect_raw', self.img_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.create_timer(1/30.0, self.publish_cycle)
        self.create_timer(1/100.0, self.check_actions)
        self.get_logger().info("--- VTAM Robot Node Ready ---")

    def img_cb(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def joint_cb(self, msg):
        try:
            self.joints = dict(zip(msg.name, msg.position))
            self.joint_vels = dict(zip(msg.name, msg.velocity))
        except Exception as e:
            self.get_logger().error(f"joint_cb: {e}")

    def encode_jp2(self, depth_img):
        if depth_img is None:
            depth_img = np.zeros((480, 640), dtype=np.uint16)
        success, buf = cv2.imencode(".jp2", depth_img.astype(np.uint16))
        return buf.tobytes() if success else b""

    def publish_cycle(self):
        if self.img is None:
            self.get_logger().warn(f"publish_cycle: Waiting for camera (step={self.step})")
            return
        if not self.joints:
            self.get_logger().warn(f"publish_cycle: Waiting for joints (step={self.step})")
            return

        img_resized = cv2.resize(self.img, (640, 480))
        _, rgb_buf = cv2.imencode('.jpg', img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        rgb_bytes = rgb_buf.tobytes()
        depth_bytes = self.encode_jp2(None)

        # Ordered joint config matching HelloStretchIdx
        robot_config = np.array([self.joints.get(j, 0.0) for j in ORDERED_JOINTS], dtype=np.float32)

        fake_vels = [0.0] * 14
        fake_vels[2] = 0.01  # liveness check

        obs_msg = {
            "rgb": rgb_bytes, "depth": depth_bytes, "joint": self.joints,
            "timestamp": time.time(), "camera_K": self.camera_K,
            "rgb_height": 480, "rgb_width": 640, "step": self.step
        }

        state_msg = {
            "control_mode": "manipulation", "at_goal": False,
            "is_homed": True, "is_runstopped": False,
            "joint_positions": robot_config.tolist(),
            "joint_velocities": fake_vels,
            "joint_efforts": [0.0] * 14,
            "ee_pose": np.eye(4), "base_pose": np.array([0.0, 0.0, 0.0]),
            "step": self.step
        }

        servo_msg = {
            "head_cam/color_image": rgb_bytes,
            "head_cam/depth_image": depth_bytes,
            "head_cam/image_scaling": 1.0,
            "head_cam/depth_scaling": 1.0,
            "head_cam/depth_camera_K": self.camera_K,
            "head_cam/pose": np.eye(4),

            "ee_cam/color_image": rgb_bytes,
            "ee_cam/depth_image": depth_bytes,
            "ee_cam/image_scaling": 1.0,
            "ee_cam/depth_scaling": 1.0,
            "ee_cam/depth_camera_K": self.camera_K,
            "ee_cam/pose": np.eye(4),

            "ee/pose": np.eye(4),
            "robot/config": robot_config,
            "step": self.step
        }

        for port, msg, name in zip([4401, 4403, 4404], [obs_msg, state_msg, servo_msg], ['obs', 'state', 'servo']):
            try:
                self.zmq_pubs[port].send(pickle.dumps(msg), flags=zmq.NOBLOCK)
            except zmq.ZMQError as e:
                self.get_logger().error(f"ZMQ Error port {port} ({name}): {e}")

        if self.step % 30 == 0:
            self.get_logger().info(f"Step {self.step} published OK")
        self.step += 1

    def check_actions(self):
        try:
            raw = self.action_sub.recv(flags=zmq.NOBLOCK)
            action = pickle.loads(raw)

            targets = None
            if isinstance(action, dict):
                if "joint" in action:
                    targets = action["joint"]
                elif "action" in action:
                    targets = action["action"]
                elif "control_mode" in action:
                    pass  # ignore mode switches
                else:
                    self.get_logger().warn(f"Unknown action keys: {list(action.keys())}")
            else:
                targets = action

            if targets is not None:
                self.execute_trajectory(targets)

        except zmq.Again:
            pass
        except Exception as e:
            self.get_logger().error(f"Action error: {e}")

    def execute_trajectory(self, targets):
        if len(targets) < 9:
            self.get_logger().warn(f"Unexpected action length: {len(targets)}")
            return

        # Base velocity
        t = Twist()
        t.linear.x = float(targets[0])
        t.angular.z = float(targets[2])
        self._cmd_vel.publish(t)

        # Arm joints
        lift       = float(targets[3])
        arm_ext    = float(targets[4])
        wrist_roll  = float(targets[5])
        wrist_pitch = float(targets[6])
        wrist_yaw   = float(targets[7])
        grip        = float(targets[8])
        arm_per_link = arm_ext / 4.0

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [
            lift,
            arm_per_link, arm_per_link, arm_per_link, arm_per_link,
            wrist_yaw, wrist_pitch, wrist_roll,
            grip
        ]
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points = [point]
        goal.trajectory = traj

        if self._action_client.server_is_ready():
            self._action_client.send_goal_async(goal)
        else:
            self.get_logger().error("Action server not ready!")


def main():
    rclpy.init()
    rclpy.spin(VTAMRobotNode())


if __name__ == '__main__':
    main()