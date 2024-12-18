#!/bin/bash
mate-terminal --tab --title="joint_action_server" -- bash -c "rosrun intera_interface joint_trajectory_action_server.py; exec bash" 
mate-terminal --tab --title="sawyer_config" -- bash -c "roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true; exec bash" 
mate-terminal --tab --title="ar_tag_detect" -- bash -c "roslaunch sawyer_full_stack sawyer_camera_track.launch; exec bash" 
mate-terminal --tab --title="dummy_main.py" -- bash -c "python src/internal/dummy_main.py; exec bash" 
mate-terminal --tab --title="main.py" -- bash -c "python src/sawyer_full_stack/main.py; exec bash"
