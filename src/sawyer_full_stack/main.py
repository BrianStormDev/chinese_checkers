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
from moveit_msgs.srv import GetPositionIKRequest
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

        # # Tuck the arm, Alan
        # tuck_positions = {
        #     'right_j0': 0,
        #     'right_j1': -0.5,
        #     'right_j2': 0,
        #     'right_j3': 1.5,
        #     'right_j4': 0,
        #     'right_j5': -0.85,
        #     'right_j6': 1.7
        # }

        # Tuck the arm, Alan
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
        print("Retrying ...")

    return trans


def convert_internal_coordinates_to_real_coordinates(x: int, y: int, transform):
    """
    Takes in internal coordinates of the board and converts them into real world coordinates
    """
    # Keep in mind the ar_tag will be in the bottom left of the board
    # We need to get the coordinates of the point closest to the ar tag
    # get the rest of the pegs with respect to that point

    # Keep in mind, these values are with respect to the base axes 
    # Real x should be negative and real y should be negative
    
    # Full x length: 58 mm
    # ar_tag_square length: 5.5 mm, half length: 2.75 mm
    # Hole radius: 4.5 mm
    # real x shift: - (58 - 2.75 - 4.5) = -50.75

    # Full y length: 27 mm
    # ar_tag_square length: 5.5 mm, half length: 2.75 mm
    # Hole radius: 4.5 mm
    # real x shift: - (27 - 2.75 - 4.5) = -19.75

    # In base coordinates
    BOTTOM_LEFT_REAL_X = - 0.0479
    BOTTOM_LEFT_REAL_Y = - 0.018

    # Y - direction
    # Difference of outer edges: 26.5 mm
    # Difference of inner edges: 11.5 mm
    # Radii of holes: 4.5 mm
    # Difference of centers: 26.5 - 4.5 - 4.5 = 17.5 -> 0.0175

    # X - direction
    # Since every x peg is 2 pegs along we divid the gap by 2
    # Difference of outer edges: 29.5 mm
    # Difference of inner edges: 11.5 mm
    # Radii of holes: 4.5 mm
    # Difference of centers: 4.5 + 11.5 +4.5 = 20.5 -> 0.0205 -> 0.01025
    REAL_SPACING_X = - 0.02
    REAL_SPACING_Y = 0.00954

    # X_Max: 0.642
    # Y_Max: 0.151

    # X_Min: 0.806
    # Y_Min: -0.078

    # X_spacing: 0.01367
    # Y_spacing: 0.00954

    # X_Max: 0.644
    # Y_Max: 0.151

    # X_Min: 0.804
    # Y_Min: -0.078

    # X_spacing: 0.01367
    # Y_spacing: 0.00954
    
    # Bottom left peg is 0, 4
    BOTTOM_LEFT_INTERNAL_X = 0
    BOTTOM_LEFT_INTERNAL_Y = 4

    # Getting the position of the AR tag wrt the base frame
    ar_tag_x = transform.transform.translation.x
    ar_tag_y = transform.transform.translation.y
    ar_tag_z = transform.transform.translation.z

    # Keep in mind, these values are with respect to the base axes
    # We first convert the internal x and y values to a real offset before added the tag offset and the offset to the (0,4) Peg
    # Keep in mind that the x and y in the board different from the x and y in the base frame
    new_x = (y - BOTTOM_LEFT_INTERNAL_Y) * REAL_SPACING_X + BOTTOM_LEFT_REAL_X + ar_tag_x
    new_y = (x - BOTTOM_LEFT_INTERNAL_X) * REAL_SPACING_Y + BOTTOM_LEFT_REAL_Y + ar_tag_y
    new_z = ar_tag_z

    # # Incase the precision is the issue
    precision = 2
    new_x = round(new_x, precision)
    new_y = round(new_y, precision)
    new_z = round(new_z, precision)

    return [new_x, new_y, new_z]

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper', calibrate=True)

    # # Amir name
    # right_gripper = robot_gripper.Gripper('stp_022312TP99620_tip_1', calibrate=True)

    return right_gripper

def control_gripper(right_gripper, open):
    """
    
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0

    # Higher values close it up
    # Lower values open it up

    # # Our gripper values
    # # Open the right gripper
    # if open:
    #     while input("Try opening the gripper: ") == "y":
    #         print('Opening gripper.')
    #         right_gripper.set_position(0.027)
    #         rospy.sleep(1)

    # # Close the right gripper
    # else:
    #     while input("Try closing the gripper: ") == "y":
    #         print('Closing gripper.')
    #         right_gripper.set_position(0.034)
    #         rospy.sleep(1)

    # Standard gripper values
    # Open the right gripper
    if open:
        while input("Try opening the gripper: ") == "y":
            print('Opening gripper.')
            right_gripper.set_position(0.027)
            rospy.sleep(1)

    # Close the right gripper
    else:
        while input("Try closing the gripper: ") == "y":
            print('Closing gripper.')
            right_gripper.set_position(0.0)
            rospy.sleep(1)    


def callback(message):
    """
    """
    # Default parameters
    ar_marker = 0

    # Unpack the message    
    start_x = message.start_x
    start_y = message.start_y
    end_x = message.end_x
    end_y = message.end_y

    rospy.logerr(f"Message recieved: {start_x}, {start_y}, {end_x}, {end_y}.")

    # Tucks the robot into a position where it can see the ar_tag
    ar_tuck()

    # Calibrates the gripper and initializes the right gripper object through which the gripper can be controlled
    # right_gripper = calibrate_gripper()
    # control_gripper(right_gripper, True)
    # control_gripper(right_gripper, False)

    # Ensure that the gripper is initially open
    # control_gripper(right_gripper, True)

    # Convert the internal points to real world points
    transform = lookup_tag(ar_marker)
    rospy.loginfo(transform.transform.position)
    start_position = convert_internal_coordinates_to_real_coordinates(start_x, start_y, transform)
    end_position = convert_internal_coordinates_to_real_coordinates(end_x, end_y, transform)
    
    rospy.logerr(f"{start_position}, {end_position}")

    regular_tuck()
    rospy.sleep(2.0)

    # Move the robot to the specified position
    move_robot(start_position, 0.125)
    move_robot(start_position, 0.1)
    rospy.sleep(1.0)

    # Close the gripper
    # control_gripper(right_gripper, False)
    rospy.sleep(1.0)

    # Move the robot to a good picking position
    regular_tuck()
    rospy.sleep(1.0)

    # Move the arm to the designated position
    move_robot(end_position, 0.125)
    move_robot(end_position, 0.1)
    rospy.sleep(1.0)

    # Open the gripper
    # control_gripper(right_gripper, True)

    # Move the arm to a spot that doesn't block the camera
    camera_tuck()
    rospy.sleep(1.0)

def move_robot(position, height_offset):
    """
    position is a list [x, y, z]
    """
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    request.ik_request.ik_link_name = "right_gripper_tip"
    request.ik_request.pose_stamped.header.frame_id = "base"
        
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = position[0]
    request.ik_request.pose_stamped.pose.position.y = position[1]
    request.ik_request.pose_stamped.pose.position.z = position[2] + height_offset
    request.ik_request.pose_stamped.pose.orientation.x = 0
    request.ik_request.pose_stamped.pose.orientation.y = 1
    request.ik_request.pose_stamped.pose.orientation.z = 0
    request.ik_request.pose_stamped.pose.orientation.w = 0
        
    try:
        # Print the response HERE
        group = MoveGroupCommander("right_arm")

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Set the planning time
        group.set_planning_time(10)

        # Set the bounds of the workspace
        group.set_workspace([-2, -2, -2, 2, 2, 2])

        # Set the tolerance in the goal final position
        group.set_goal_position_tolerance(0.00001)

        group.set_num_planning_attempts(5)  # Try multiple times
        group.set_max_velocity_scaling_factor(0.1)  # Slow down for precision
        group.set_max_acceleration_scaling_factor(0.05)  # Reduce jerks

        # Plan IK
        plan = group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
        
        # Execute IK if safe
        if user_input == 'y':
            group.execute(plan[1])
            rospy.sleep(1.0)
            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('move_board_subscriber')
    rospy.Subscriber('game_move', BoardMove, callback)
    rospy.spin()
