#!/bin/bash
mate-terminal \
    --tab --title="joint_action_server" -- bash -c "rosrun intera_interface joint_trajectory_action_server.py; exec bash" \
    --tab --title="sawyer_config" -- bash -c "roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true; exec bash" \
    --tab --title="ar_tag_detect" -- bash -c "roslaunch sawyer_full_stack sawyer_camera_track.launch; exec bash" \
    --tab --title="main.py" -- bash -c "python src/sawyer_full_stack/main.py; exec bash"
wait 