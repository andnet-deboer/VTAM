#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    """
    A ROS 2 node that subscribes to joint states and prints joint information.
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('joint_state_subscriber')

        # Create a subscription to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState,                    # Message type
            '/joint_states',               # Topic name
            self.joint_state_callback,     # Callback function
            10                            # Queue size
        )

        # Prevent unused variable warning
        self.subscription

        # Log that the node has started
        self.get_logger().info('Joint State Subscriber node has started!')
        self.get_logger().info('Listening for joint states on /joint_states topic...')

    def joint_state_callback(self, msg):
        """
        Callback function that gets called whenever a new JointState message is received.

        Args:
            msg (JointState): The received joint state message
        """
        # Log basic information about the message
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

        # Print detailed information about each joint
        self.get_logger().info('Joint Information:')
        for i, joint_name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 'N/A'
            velocity = msg.velocity[i] if i < len(msg.velocity) else 'N/A'
            effort = msg.effort[i] if i < len(msg.effort) else 'N/A'

            self.get_logger().info(
                f'  {joint_name}: pos={position:.4f}, vel={velocity:.4f}, effort={effort:.4f}'
            )

        self.get_logger().info('---')

def main(args=None):
    """
    Main function to initialize ROS 2, create the node, and spin.
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    joint_state_subscriber = JointStateSubscriber()

    try:
        # Spin the node to keep it alive and processing callbacks
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        joint_state_subscriber.get_logger().info('Shutting down joint state subscriber...')
    finally:
        # Clean up
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()