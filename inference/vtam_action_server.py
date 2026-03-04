import zmq
import rclpy
import threading
import json
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class VTAMActionServer(Node):
    def __init__(self):
        super().__init__('vtam_action_server')
        self._cmd_vel = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self._arm_client = ActionClient(self, FollowJointTrajectory,
                                        '/stretch_controller/follow_joint_trajectory')
        ctx = zmq.Context()
        self._sock = ctx.socket(zmq.PULL)
        self._sock.bind("tcp://*:4405")
        threading.Thread(target=self._recv_loop, daemon=True).start()
        self.get_logger().info("VTAM Action Server ready on port 4405")

    def _recv_loop(self):
        while True:
            msg = json.loads(self._sock.recv_string())
            t = Twist()
            t.linear.x = float(msg['base_trans_vel'])
            t.angular.z = float(msg['base_rot_vel'])
            self._cmd_vel.publish(t)
            arm = msg['arm']
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [
                'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
                'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'gripper_aperture'
            ]
            pt = JointTrajectoryPoint()
            arm_per = arm[1] / 4.0
            pt.positions = [arm[0], arm_per, arm_per, arm_per, arm_per,
                            arm[2], arm[3], arm[4], arm[5]]
            pt.time_from_start.nanosec = 100_000_000
            goal.trajectory.points = [pt]
            self._arm_client.send_goal_async(goal)

rclpy.init()
node = VTAMActionServer()
rclpy.spin(node)