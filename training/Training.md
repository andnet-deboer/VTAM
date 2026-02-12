#### Single episode, with MuJoCo validation
python3 bag_to_zarr.py --bag ~/bags/episode_001 --output dataset.zarr

#### Single episode, skip MuJoCo (faster, IK only)
python3 bag_to_zarr.py --bag ~/bags/episode_001 --output dataset.zarr --no-mujoco

#### Multi-episode (one bag per episode)
python3 bag_to_zarr.py --bag-dir ~/bags/ --output dataset.zarr



ros2 launch stretch_core d405_basic.launch.py serial_no:=_128422270608

 ros2 launch realsense2_camera rs_launch.py camera_name:=camera namespace:=camera enable_depth:=false

 stretch_configure_tool.py #find robot hardware

# Update the core URDF tools
python3 -m pip install -U hello-robot-stretch-urdf

# Pull the latest ROS 2 description changes
cd ~/ament_ws/src/stretch_ros2
git pull

stretch_urdf_ros_update.py --ros2_rebuild