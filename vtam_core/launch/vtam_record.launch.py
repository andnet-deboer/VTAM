import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('vtam_core')
    realsense_share = get_package_share_directory('realsense2_camera')
    stretch_core_share = get_package_share_directory('stretch_core')
    
    urdf_path = '/home/leogray/VTAM/training/stretch_base_rotation_ik_with_fixed_wrist.urdf'
    if not os.path.exists(urdf_path):
        urdf_path = os.path.join(get_package_share_directory('stretch_description'), 'urdf', 'stretch.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    # --- Camera 1: D405 (Arm) ---
    camera_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(stretch_core_share, 'launch', 'd405_basic.launch.py')]),
        launch_arguments={
            'serial_no': '_128422270608',
            'camera_name': 'camera_arm'
        }.items()
    )

    # --- Camera 2: Head Camera (D435i) ---
    camera_head = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(realsense_share, 'launch', 'rs_launch.py')]),
                launch_arguments={
                    'serial_no': '_239722072992',
                    'camera_name': 'camera_head',
                    'namespace': 'camera_head',
                    'device_type': 'd435',   # ADD THIS LINE
                    'enable_depth': 'false'
                }.items()
            )
        ]
    )

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    umi_detector_node = Node(
        package='vtam_core',
        executable='umi_detector_node',
        output='screen',
        remappings=[
            ('/camera/camera/color/image_raw', '/camera_head/camera_head/color/image_raw'),
            ('/camera/camera/color/camera_info', '/camera_head/camera_head/color/camera_info')
        ]
    )

    eflesh_node = Node(package='vtam_core', executable='eflesh_node', output='screen')
    umi_gripper_node = Node(package='vtam_core', executable='umi_gripper_node', output='screen')

    # Delayed start for gripper control hardware
    delayed_gripper = TimerAction(period=8.0, actions=[umi_gripper_node])

    return LaunchDescription([
        camera_arm,
        camera_head,
        robot_state_publisher_node,
        umi_detector_node,
        eflesh_node,
        delayed_gripper
    ])