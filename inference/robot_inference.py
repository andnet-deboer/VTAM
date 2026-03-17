import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import zmq, pickle, cv2, numpy as np, math
from sensor_msgs.msg import Image, JointState
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
    'joint_gripper_finger_left'
]

# Consumed by replay_demo.py seed_from_robot_state()
# Indices: 0=base_x, 1=base_y, 2=base_theta, 3=lift, 4=arm_l0,
#          5=gripper_finger_right, 6=wrist_roll, 7=wrist_pitch, 8=wrist_yaw
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
        self.base_x = 0.0
        self.base_theta = 0.0
        self.step = 0

        self.ctx = zmq.Context()

        # Port 4401: obs (rgb + joints)  inference mode
        self.zmq_obs = self.ctx.socket(zmq.PUB)
        self.zmq_obs.bind("tcp://*:4401")

        # Port 4403: state (joint_positions) get_robot_state()
        self.zmq_state = self.ctx.socket(zmq.PUB)
        self.zmq_state.bind("tcp://*:4403")

        # Port 4402: receive actions from replay_demo.py
        self.action_sub = self.ctx.socket(zmq.SUB)
        self.action_sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.action_sub.setsockopt(zmq.CONFLATE, 1)
        self.action_sub.bind("tcp://*:4402")

        self.bridge = CvBridge()

        self._cmd_vel = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')

        self.create_subscription(Image, '/gripper_camera/color/image_rect_raw', self.img_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.create_timer(1/30.0, self.publish_cycle)
        self.create_timer(1/100.0, self.check_actions)
        self.get_logger().info("--- VTAM Robot Node Ready (with odometry) ---")

    def img_cb(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"img_cb: {e}")

    def joint_cb(self, msg):
        try:
            self.joints = dict(zip(msg.name, msg.position))
        except Exception as e:
            self.get_logger().error(f"joint_cb: {e}")

    def odom_cb(self, msg):
        self.base_x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.base_theta = math.atan2(siny, cosy)

    def publish_cycle(self):
        if self.img is None:
            self.get_logger().warn(f"publish_cycle: Waiting for camera (step={self.step})")
            return
        if not self.joints:
            self.get_logger().warn(f"publish_cycle: Waiting for joints (step={self.step})")
            return

        img_resized = cv2.resize(self.img, (640, 480))
        _, rgb_buf = cv2.imencode('.jpg', img_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 85])

        obs_msg = {
            "rgb": rgb_buf.tobytes(),
            "joint": self.joints,
        }

        robot_config = np.array([
            self.base_x,                                          # index 0
            0.0,                                                  # index 1 (base_y, unused)
            self.base_theta,                                      # index 2
            self.joints.get('joint_lift', 0.0),                   # index 3
            self.joints.get('joint_arm_l0', 0.0),                 # index 4
            self.joints.get('joint_gripper_finger_right', 0.0),   # index 5
            self.joints.get('joint_wrist_roll', 0.0),             # index 6
            self.joints.get('joint_wrist_pitch', 0.0),            # index 7
            self.joints.get('joint_wrist_yaw', 0.0),              # index 8
        ], dtype=np.float32)

        state_msg = {
            "joint_positions": robot_config.tolist(),
        }

        try:
            self.zmq_obs.send(pickle.dumps(obs_msg), flags=zmq.NOBLOCK)
            self.zmq_state.send(pickle.dumps(state_msg), flags=zmq.NOBLOCK)
        except zmq.ZMQError as e:
            self.get_logger().error(f"ZMQ send error: {e}")

        if self.step % 30 == 0:
            self.get_logger().info(f"Step {self.step} odom: x={self.base_x:.4f} theta={self.base_theta:.4f}")
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

        # Base velocity — dampened to prevent overshoot
        t = Twist()
        t.linear.x = float(targets[0])
        t.angular.z = float(targets[2])
        self._cmd_vel.publish(t)

        # Arm joints
        lift        = float(targets[3])
        arm_ext     = float(targets[4])
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