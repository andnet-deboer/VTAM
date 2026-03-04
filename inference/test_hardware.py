import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import threading
import sys

JOINT_NAMES = [
    'joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
    'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'gripper_aperture'
]

class EasyTester(Node):
    def __init__(self):
        super().__init__('easy_tester')
        self._cmd_vel = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        
        # Virtual state tracker (Starts at a safe height of 0.2m)
        self.arm_state = {
            'lift': 0.2, 'ext': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'grip': 0.0
        }
        
        self.get_logger().info("Waiting for Action Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected! Ready to move.")

    def set_joint(self, joint_name, value):
        # Update our virtual state
        self.arm_state[joint_name] = float(value)
        
        goal = FollowJointTrajectory.Goal()
        # Assign directly to the goal object!
        goal.trajectory.joint_names = JOINT_NAMES
        
        pt = JointTrajectoryPoint()
        arm_per = self.arm_state['ext'] / 4.0
        
        pt.positions = [
            self.arm_state['lift'], 
            arm_per, arm_per, arm_per, arm_per,
            self.arm_state['yaw'], self.arm_state['pitch'], self.arm_state['roll'], self.arm_state['grip']
        ]
        
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        goal.trajectory.points = [pt]
        
        self.get_logger().info(f"Moving [{joint_name}] to {value}...")
        self._action_client.send_goal_async(goal)
def input_loop(tester_node):
    print("\n--- EASY SINGLE-JOINT TESTER ---")
    print("Commands: lift, ext, yaw, pitch, roll, grip <value>")
    print("Example: ext 0.15")
    
    valid_joints = ['lift', 'ext', 'yaw', 'pitch', 'roll', 'grip']
    
    while rclpy.ok():
        try:
            cmd_in = input("\ncmd> ").strip().lower().split()
            if not cmd_in: continue
            
            cmd = cmd_in[0]
            if cmd == 'quit' or cmd == 'exit':
                rclpy.shutdown()
                sys.exit(0)
                
            elif cmd in valid_joints:
                if len(cmd_in) != 2:
                    print(f"Error: '{cmd}' needs a number.")
                    continue
                tester_node.set_joint(cmd, cmd_in[1])
            else:
                print(f"Unknown command. Try: {', '.join(valid_joints)}")
                
        except Exception as e:
            print(f"Error: {e}")

def main():
    rclpy.init()
    tester = EasyTester()
    threading.Thread(target=input_loop, args=(tester,), daemon=True).start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
