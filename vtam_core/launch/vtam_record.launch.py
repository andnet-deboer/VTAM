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
    
    # 1. Load Standard URDF
    urdf_path = os.path.join(get_package_share_directory('stretch_description'), 'urdf', 'stretch.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    # 2. Wrist Camera (D405)
    # We disable TF publishing here to avoid conflicts, then bridge it manually below.
    camera_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(stretch_core_share, 'launch', 'd405_basic.launch.py')]),
        launch_arguments={
            'serial_no': '_128422270608',
            'camera_name': 'camera_arm',
            'publish_tf': 'false'
        }.items()
    )

    # 3. Head Camera (D435i)
    # Attaches internally to 'camera_bottom_screw_frame' which is in the URDF.
    camera_head = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(realsense_share, 'launch', 'rs_launch.py')]),
                launch_arguments={
                    'serial_no': '_239722072992',
                    'camera_name': 'camera',  
                    'namespace': 'camera',     
                    'device_type': 'd435',  
                    'enable_depth': 'false',
                    'base_frame_id': 'camera_bottom_screw_frame',
                    'publish_tf': 'false' 
                }.items()
            )
        ]
    )

    # 4. Robot State Publisher
    # This reads the /joint_states from vtam_robot_node and updates the TF tree
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    # 5. Wrist Camera Bridge
    # Connects the camera driver frame (camera_arm_link) to the robot gripper (link_aruco_d405)
    wrist_camera_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wrist_camera_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'link_aruco_d405', 'camera_arm_link']
    )

    # 6. VTAM Robot Node (THE DRIVER)
    # Now publishes joint states!
    vtam_robot_node = TimerAction(period=8.0, actions=[Node(
        package='vtam_core',
        executable='vtam_robot_node',
        output='screen',
    )])

    # 7. UMI Detector & Sensor
    umi_detector_node = Node(
        package='vtam_core',
        executable='umi_detector_node',
        output='screen',
        parameters=[{'camera_optical_frame': 'camera_color_optical_frame'}]
    )
    
    eflesh_node = Node(
        package='vtam_core',
        executable='eflesh_node'
    )
    
    return LaunchDescription([
        camera_arm,
        camera_head,
        robot_state_publisher_node,
        wrist_camera_bridge,
        umi_detector_node,
        eflesh_node,
        vtam_robot_node,
    ])