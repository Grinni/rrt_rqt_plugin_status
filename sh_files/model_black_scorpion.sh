#!/bin/bash -e
source /opt/ros/hydro/setup.sh
export ROS_PACKAGE_PATH=/home/rrt/ros_workspace:/home/rrt/catkin_ws:$ROS_PACKAGE_PATH
source /home/rrt/ros_workspace/rrt_rqt_plugin_status/sh_files/ros-network.sh
export ROS_MASTER_URI=http://rrt-devil:11311

roslaunch black_scorpion_description black_scorpion_description.launch 

 
