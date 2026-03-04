#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/hello-robot/ament_ws/install/setup.bash

export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export HELLO_FLEET_ID=stretch-se3-3047

echo "--- STARTING INTEGRATED VTAM ROBOT NODE ---"
python3 /home/hello-robot/VTAM/inference/vtam_robot_node.py
