#!/bin/bash

rosrun intera_interface joint_trajectory_action_server.py &
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true &
roslaunch sawyer_full_stack sawyer_camera_track.launch &
roslaunch realsense2_camera rs_camera.launch &
python src/sawyer_full_stack/main.py 

wait 