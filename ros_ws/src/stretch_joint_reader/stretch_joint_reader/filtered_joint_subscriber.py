#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FilteredJointStateSubscriber(Node):
    """
    An enhanced ROS 2 node that subscribes to joint states and filters for specific joints.
    """

    def __init__(self):
        super().__init__('filtered_joint_state_subscriber')

        # Define which joints we're interested in
        self.joints_of_interest = [
            'joint_lift',
            'wrist_extension',
            'joint_wrist_yaw',
            'joint_gripper_finger_left',
            'joint_gripper_finger_right'
        ]

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create a timer to periodically print a summary
        self.timer = self.create_timer(5.0, self.print_summary)

        # Store the latest joint states
        self.latest_joint_states = {}

        self.get_logger().info('Filtered Joint State Subscriber started!')
        self.get_logger().info(f'Monitoring joints: {", ".join(self.joints_of_interest)}')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages."""
        # Update our stored joint states
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joints_of_interest:
                self.latest_joint_states[joint_name] = {
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def print_summary(self):
        """Print a summary of current joint states."""
        if not self.latest_joint_states:
            self.get_logger().warn('No joint state data received yet...')
            return

        self.get_logger().info('=== Joint State Summary ===')
        for joint_name in self.joints_of_interest:
            if joint_name in self.latest_joint_states:
                state = self.latest_joint_states[joint_name]
                self.get_logger().info(
                    f'{joint_name:25}: {state["position"]:8.4f} rad/m'
                )
        self.get_logger().info('===========================')

def main(args=None):
    rclpy.init(args=args)
    node = FilteredJointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()