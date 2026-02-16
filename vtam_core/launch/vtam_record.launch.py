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
    
    urdf_path = os.path.join(
        get_package_share_directory('stretch_description'), 
        'urdf', 
        'stretch.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    # D405 (Wrist Camera)
    camera_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(stretch_core_share, 'launch', 'd405_basic.launch.py')
        ]),
        launch_arguments={
            'serial_no': '_128422270608',
            'camera_name': 'camera_arm',
            'rgb_camera.enable_auto_exposure': 'true',
            'rgb_camera.auto_exposure_priority': 'false',
        }.items()
    )
    # Head Camera (D435i) - publishes TF starting from camera_bottom_screw_frame
    camera_head = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(realsense_share, 'launch', 'rs_launch.py')
                ]),
                launch_arguments={
                    'serial_no': '_239722072992',
                    'camera_name': 'camera',
                    'device_type': 'd435',
                    'enable_depth': 'false',
                    'camera_namespace': '',
                    'base_frame_id': 'camera_bottom_screw_frame',
                    'unite_imu_method': '0',
                    'rgb_camera.color_profile': '1280x720x60',
                }.items()
            )
        ]
    )

    # Robot state publisher - publishes TF from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    # UMI detector
    umi_detector_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='vtam_core',
            executable='umi_detector_node',
            output='screen',
            parameters=[{
                'camera_optical_frame': 'camera_color_optical_frame'
            }]
        )]
    )

    # Robot control loop
    vtam_robot_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='vtam_core',
            executable='vtam_robot_node',
            output='screen',
        )]
    )

    # eFlesh sensor node
    eflesh_node = Node(
        package='vtam_core',
        executable='eflesh_node',
        output='screen'
    )

    # record node
    record_node = Node(
        package='vtam_core',
        executable='record_node',
        output='screen'
    )


    
    return LaunchDescription([
        robot_state_publisher_node,
        camera_arm,
        camera_head,
        umi_detector_node,
        eflesh_node,
        vtam_robot_node,
        record_node   
        ])