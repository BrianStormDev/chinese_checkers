#!/usr/bin/env python
import rospy.logger_level_service_caller

import rospy
import tf2_ros

# Added dependencies for project
from intera_interface import gripper as robot_gripper
from internal.msg import BoardMove

# Tuck dependencies
from intera_core_msgs.msg import HeadPanCommand
from intera_interface import Limb

# Move dependencies
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

def camera_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to camera_tuck the arm? (y/n): ') == 'y':
        # Set the head pan
        target_pan = 0
        speed_ratio = 1
        pan_mode = 1

        # Tuck the arm
        tuck_positions = {
            'right_j0': 0.0,
            'right_j1': -1.25,
            'right_j2': 0.0,
            'right_j3': 1.5,
            'right_j4': 0.0,
            'right_j5': -0.5,
            'right_j6': 1.7
        }

        """
        Publishes a command to control the Sawyer robot's head pan.
        """
        pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
        rospy.loginfo("Waiting for publisher to register...")
        rospy.sleep(1)  # Give some time for the publisher to register

        command = HeadPanCommand(target=target_pan, speed_ratio=speed_ratio, pan_mode=pan_mode)
        rospy.loginfo(f"Publishing head pan command: {command}")
        pub.publish(command)
        rospy.sleep(1)  # Allow some time for the message to be processed

        """
        Commands the Sawyer robot's arm to a tucked position using joint positions.
        """
        limb = Limb("right")  # Use "right" since Sawyer has only one arm
        rospy.loginfo(f"Moving arm to tuck position: {tuck_positions}")
        limb.move_to_joint_positions(tuck_positions)
    else:
        print('Canceled. Not tucking the arm.')

def regular_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to regular_tuck the arm? (y/n): ') == 'y':
        # Set the head pan
        target_pan = 0
        speed_ratio = 1
        pan_mode = 1

        # Tuck the arm
        tuck_positions = {
            'right_j0': 0.0,
            'right_j1': -1.0,
            'right_j2': 0.0,
            'right_j3': 1.0,
            'right_j4': 0.0,
            'right_j5': 1.6,
            'right_j6': 1.7
        }

        """
        Publishes a command to control the Sawyer robot's head pan.
        """
        pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
        rospy.loginfo("Waiting for publisher to register...")
        rospy.sleep(1)  # Give some time for the publisher to register

        command = HeadPanCommand(target=target_pan, speed_ratio=speed_ratio, pan_mode=pan_mode)
        rospy.loginfo(f"Publishing head pan command: {command}")
        pub.publish(command)
        rospy.sleep(1)  # Allow some time for the message to be processed

        """
        Commands the Sawyer robot's arm to a tucked position using joint positions.
        """
        limb = Limb("right")  # Use "right" since Sawyer has only one arm
        rospy.loginfo(f"Moving arm to tuck position: {tuck_positions}")
        limb.move_to_joint_positions(tuck_positions)
    else:
        print('Canceled. Not tucking the arm.')

def ar_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to ar_tuck the arm? (y/n): ') == 'y':
        # Set the head pan
        target_pan = 0
        speed_ratio = 1
        pan_mode = 1

        # Joints 1, 3, 5 are the joints to be changed
        # 1 - 5 = -0.5
        # 1 + 3 = 0.5

        # # Tuck the arm, Alice
        # tuck_positions = {
        #     'right_j0': 0.0,
        #     'right_j1': 0.0,
        #     'right_j2': 0,
        #     'right_j3': 0.5,
        #     'right_j4': 0,
        #     'right_j5': -0.5,
        #     'right_j6': 1.7
        # }

        # # Tuck the arm, Azula
        # tuck_positions = {
        #     'right_j0': 0,
        #     'right_j1': -1,
        #     'right_j2': 0,
        #     'right_j3': 1.5,
        #     'right_j4': 0,
        #     'right_j5': -0.45,
        #     'right_j6': 1.7
        # }

        # Tuck the arm, Alan
        tuck_positions = {
            'right_j0': 0,
            'right_j1': -0.5,
            'right_j2': 0,
            'right_j3': 1.25,
            'right_j4': 0,
            'right_j5': -0.75,
            'right_j6': 1.7
        }

        """
        Publishes a command to control the Sawyer robot's head pan.
        """
        pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
        rospy.loginfo("Waiting for publisher to register...")
        rospy.sleep(1)  # Give some time for the publisher to register

        command = HeadPanCommand(target=target_pan, speed_ratio=speed_ratio, pan_mode=pan_mode)
        rospy.loginfo(f"Publishing head pan command: {command}")
        pub.publish(command)
        rospy.sleep(1)  # Allow some time for the message to be processed

        """
        Commands the Sawyer robot's arm to a tucked position using joint positions.
        """
        limb = Limb("right")  # Use "right" since Sawyer has only one arm
        rospy.loginfo(f"Moving arm to tuck position: {tuck_positions}")
        limb.move_to_joint_positions(tuck_positions)
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    tfBuffer = tf2_ros.Buffer()
    tfLis = tf2_ros.TransformListener(tfBuffer)

    try:
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
    return trans

def convert_internal_coordinates_to_real_coordinates(x: int, y: int, trans):
    """
    Takes in internal coordinates of the board and converts them into real world coordinates
    """
    # Keep in mind the ar_tag will be in the bottom left of the board
    # We need to get the coordinates of the point closest to the ar tag
    # get the rest of the pegs with respect to that point

    # Keep in mind, these values are with respect to the base axes 
    # Real x should be negative and real y should be negative

    # Bottom left peg is 0, 4
    INTERNAL_BOTTOM_LEFT_X = 0
    INTERNAL_BOTTOM_LEFT_Y = 4

    # The offset of the bottom left piece
    REAL_BOTTOM_LEFT_X = - 0.0476
    REAL_BOTTOM_LEFT_Y = - 0.0218

    # The spacing of the pieces
    REAL_SPACING_X = -0.01963
    REAL_SPACING_Y = 0.00937

    REAL_BOTTOM_LEFT_X, REAL_BOTTOM_LEFT_Y, REAL_SPACING_X, REAL_SPACING_Y = [0.1449, 0.1824, -0.0244, -0.0081]

    # Getting the position of the AR tag wrt the base frame
    ar_tag_x = trans.transform.translation.x
    ar_tag_y = trans.transform.translation.y
    ar_tag_z = trans.transform.translation.z

    # Keep in mind, these values are with respect to the base axes
    # We first convert the internal x and y values to a real offset before added the tag offset and the offset to the (0,4) Peg
    # Keep in mind that the x and y in the board different from the x and y in the base frame
    new_x = (y - INTERNAL_BOTTOM_LEFT_Y) * REAL_SPACING_X + REAL_BOTTOM_LEFT_X + ar_tag_x
    new_y = (x - INTERNAL_BOTTOM_LEFT_X) * REAL_SPACING_Y + REAL_BOTTOM_LEFT_Y + ar_tag_y
    new_z = ar_tag_z

    return [new_x, new_y, new_z]

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper', calibrate=True)
    return right_gripper

def control_gripper(right_gripper, open):
    """
    Control the gripper
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0

    # Higher values close it up
    # Lower values open it up

    # Reversed grippers
    # Open the right gripper
    if open:
        while input("Try opening the gripper: ") == "y":
            print('Opening gripper.')
            right_gripper.set_position(0.022)
            rospy.sleep(1)

    # Close the right gripper
    else:
        while input("Try closing the gripper: ") == "y":
            print('Closing gripper.')
            right_gripper.set_position(0.033)
            rospy.sleep(1)

# # Callback function for easy gripper calibration
# def callback(message):
#     right_gripper = calibrate_gripper()
#     control_gripper(right_gripper, True)
#     control_gripper(right_gripper, False)

def callback(message):
    """
    Main loop
    """
    # Default parameters
    ar_marker = 0
    TOP_HEIGHT = 0.125
    PICKUP_HEIGHT = 0.057

    # Unpack the message    
    start_x = message.start_x
    start_y = message.start_y
    end_x = message.end_x
    end_y = message.end_y

    # Log the received message
    rospy.loginfo(f"Message recieved: {start_x}, {start_y}, {end_x}, {end_y}.")

    # Tucks the robot into a position where it can see the ar_tag
    ar_tuck()

    # Calibrates the gripper and initializes the right gripper object through which the gripper can be controlled
    right_gripper = calibrate_gripper()

    # Ensure that the gripper is initially open
    control_gripper(right_gripper, True)

    # Convert the internal points to real world points
    trans = lookup_tag(ar_marker)
    rospy.loginfo(trans.transform.translation) # Log the position of ar tag for calibration
    start_position = convert_internal_coordinates_to_real_coordinates(start_x, start_y, trans)
    end_position = convert_internal_coordinates_to_real_coordinates(end_x, end_y, trans)
    
    # Log the start and end position in the base frame
    rospy.loginfo(f"{start_position}, {end_position}")

    # Tuck the robot into a position that it can do IK with easily
    regular_tuck()
    move_robot(342, 0.01)
    move_robot(342, 0)

    # # Pick up the ball
    move_robot(start_position, TOP_HEIGHT)
    move_robot(start_position, PICKUP_HEIGHT)
    control_gripper(right_gripper, False)
    move_robot(start_position, TOP_HEIGHT)

    # Move the robot to a good picking position
    regular_tuck()

    # Drop the ball in the designated position
    move_robot(end_position, TOP_HEIGHT)
    # move_robot(end_position, PICKUP_HEIGHT)
    control_gripper(right_gripper, True)
    move_robot(end_position, TOP_HEIGHT)

    # Move the arm to a spot that doesn't block the camera
    camera_tuck()

def move_robot(position, height_offset):
   # Initialize move_group for the Sawyer arm
    group = MoveGroupCommander("right_arm")

    # Set up planner for precision and shortest path
    group.set_planner_id("PRMstarConfigDefsdfault") # Optimal Smooth Paths
    group.set_planning_time(10)  # Increase planning time for more complex paths
    group.set_goal_tolerance(0.0001)  # Set the joint, position and orientation goal tolerances simultaneously 
    group.set_num_planning_attempts(5)  # Try multiple times
    group.set_max_velocity_scaling_factor(0.3)  # Slow down for precision
    group.set_max_acceleration_scaling_factor(0.005)  # Reduce jerks
    group.set_workspace([0.4, -0.5, -0.17, 1, 0.6, 2]) # Setting the workspace 

    # Define goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "base"
    goal_pose.pose.position.x = position[0]
    goal_pose.pose.position.y = position[1]
    goal_pose.pose.position.z = position[2] + height_offset
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 1.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 0.0

    group.set_pose_target(goal_pose)

    # Plan path
    plan = group.plan()
    user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
    
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])
        rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node('move_board_subscriber')
    rospy.Subscriber('game_move', BoardMove, callback)
    rospy.spin()