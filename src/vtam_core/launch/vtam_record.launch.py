#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, RegisterEventHandler, Shutdown, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('vtam_core')
    realsense_share = get_package_share_directory('realsense2_camera')
    stretch_core_share = get_package_share_directory('stretch_core')

    # Stretch driver (handles hardware, joint_states, robot_state_publisher, TF)
    stretch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(stretch_core_share, 'launch', 'stretch_driver.launch.py')
        ]),
    )

    # Wrist Camera (D405)
    camera_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(stretch_core_share, 'launch', 'd405_basic.launch.py')
        ]),
        launch_arguments={
            'serial_no': '_128422270608',
            'camera_name': 'camera_arm',
            'rgb_camera.color_profile': '640x480x30',
            'rgb_camera.enable_auto_exposure': 'true',
            'rgb_camera.auto_exposure_priority': 'false',
            'rgb_camera.brightness': '50',
            'rgb_camera.contrast': '50',
            'rgb_camera.gamma': '300',
            'rgb_camera.sharpness': '50',
            'rgb_camera.saturation': '64',
        }.items(),
    )

    # Head Camera (D435i)
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
                    'rgb_camera.color_profile': '1280x720x30',       # Back to fast compute
                    'rgb_camera.enable_auto_exposure': 'false',     # Kill the hunting
                    'rgb_camera.exposure': '166',                   # The magic 60Hz safe number
                    'rgb_camera.gain': '128',                       # Boost brightness to compensate
                    'depth_module.emitter_enabled': '0',            # Keep IR laser off
                }.items()
            )
        ]
    )

    # UMI detector
    # umi_detector_node = TimerAction(
    #     period=4.0,
    #     actions=[Node(
    #         package='vtam_core',
    #         executable='umi_detector_node',
    #         output='screen',
    #         parameters=[{
    #             'camera_optical_frame': 'camera_color_optical_frame'
    #         }]
    #     )]
    # )

    # Gripper controller
    gripper_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='vtam_core',
            executable='umi_gripper_node',
            output='screen',
        )]
    )

    # eFlesh sensor node
    eflesh_node = Node(
        package='vtam_core',
        executable='eflesh_node',
        output='screen'
    )

    terminate_on_eflesh_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=eflesh_node,
            on_exit=[
                LogInfo(msg="\n" + "="*50 +
                            "\nERROR: Adafruit QT Py (UMI Sensor) not found on /dev/serial/by-id/..." +
                            "\nCHECK: Is the USB cable plugged in?" +
                            "\n" + "="*50 + "\n"),
                Shutdown(reason='Hardware missing')
            ]
        )
    )

    # Record node
    topics_config = os.path.join(pkg_share, 'config', 'record.yaml')
    record_node = Node(
        package='vtam_core',
        executable='record_node',
        parameters=[topics_config, {'demo_name': 'default_demo'}],
    )

    # # Head tracker service
    # head_tracker_node = TimerAction(
    #     period=7.0,
    #     actions=[Node(
    #         package='vtam_core',
    #         executable='head_tracker_node',
    #         output='screen',
    #     )]
    # )

    return LaunchDescription([
        stretch_driver,
        camera_arm,
        camera_head,
        # umi_detector_node,
        eflesh_node,
        terminate_on_eflesh_exit,
        gripper_node,
        record_node,
        # head_tracker_node,
    ])