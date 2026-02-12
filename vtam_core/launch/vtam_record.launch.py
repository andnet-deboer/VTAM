import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Paths ---
    # Path to your custom URDF in the training folder
    urdf_path = '/home/leogray/VTAM/training/stretch_base_rotation_ik_with_fixed_wrist.urdf'

    # Check if the file exists to prevent launch failure
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at {urdf_path}")
        # Fallback to system URDF if yours is missing
        urdf_path = os.path.join(
            get_package_share_directory('stretch_description'),
            'urdf', 'stretch.urdf'
        )

    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    # --- 2. Node Configurations ---
    
    # The Engine: Converts JointStates -> TF Tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False,
            'publish_frequency': 30.0
        }]
    )

    # The Driver: Direct Hardware + JointState Publisher
    umi_gripper_node = Node(
        package='vtam_core',
        executable='umi_gripper_node',
        name='umi_gripper_node',
        output='screen',
        parameters=[{
            'threshold_percent': 0.12,
            'smoothing': 0.2,
            'curve_exponent': 0.6
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        umi_gripper_node
    ])