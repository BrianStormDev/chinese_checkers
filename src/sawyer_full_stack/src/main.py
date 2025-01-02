#!/usr/bin/env python
import rospy
import tf2_ros
from intera_interface import gripper as robot_gripper # Gripper dependency
from internal.msg import BoardMove # Message dependency

# Tuck dependencies
from intera_core_msgs.msg import HeadPanCommand
from intera_interface import Limb

# Move dependencies
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

# How long to sleep in between actions
SLEEP = 1.0

# Custom tuck angles
CAMERA_TUCK = [0.0, -1.25, 0.0, 1.5, 0.0, -0.5, 1.7]
REGULAR_TUCK = [0.0, -1.0, 0.0, 1.0, 0.0, 1.6, 1.7]
ALICE_AR_TUCK = [0.0, -0.5, 0.0, 1.25, 0.0, -0.75, 1.7]

# Height offsets for picking up the balls
TOP_HEIGHT = 0.099
PICKUP_HEIGHT = 0.0835

# The ar tag number
AR_MARKER = 0

def tuck(angles, wait_for_user):
    """
    Tuck the robot arm to a custom position
    angles: list of angles (floats)
    wait_for_user: bool on whether or not the movement should wait on user input or not
    """
    if not wait_for_user or input('Would you like to tuck the arm? (y/n): ') == 'y':
        # Set the head pan
        target_pan = 0
        speed_ratio = 1
        pan_mode = 1
        
        # Set the tuck_position
        tuck_positions = {
            'right_j0': angles[0],
            'right_j1': angles[1],
            'right_j2': angles[2],
            'right_j3': angles[3],
            'right_j4': angles[4],
            'right_j5': angles[5],
            'right_j6': angles[6]
        }
        
        # Publishes a command to control the Sawyer robot's head pan.
        pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=10)
        rospy.loginfo("Waiting for publisher to register...")
        command = HeadPanCommand(target=target_pan, speed_ratio=speed_ratio, pan_mode=pan_mode)
        rospy.loginfo(f"Publishing head pan command: {command}")
        pub.publish(command)
        rospy.sleep(SLEEP)  
        
        # Commands the Sawyer robot's arm to a tucked position using joint positions.
        limb = Limb("right")  # Use "right" since Sawyer has only one arm
        rospy.loginfo(f"Moving arm to tuck position: {tuck_positions}")
        limb.move_to_joint_positions(tuck_positions)
        rospy.sleep(SLEEP)
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number: int):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    tag_number: int
    Returns: geometry_msgs/Vector3
    """
    tfBuffer = tf2_ros.Buffer()
    tfLis = tf2_ros.TransformListener(tfBuffer)

    try:
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        trans = tfBuffer.lookup_transform('base', f'ar_marker_{tag_number}', rospy.Time(0), rospy.Duration(10.0))
        print("The tag was found!")
        return trans.transform.translation
    except Exception as e:
        print(e)
        print("The tag was not found!")

def convert_internal_coordinates_to_real_coordinates(x: int, y: int, trans):
    """
    Takes the position of a piece in internal coordinates of the board and converts them into real world coordinates with
    respect to the Saywer's base frame.
    x: int, internal x
    y: int, internal y
    trans: geometry_msgs/TransformStamped.msg
    """
    # Keep in mind the ar_tag will be in the bottom left of the board
    # We need to get the coordinates of the point closest to the ar tag
    # and then get the rest of the pegs with respect to that point

    # Bottom left peg is at internal coodinate (0, 4)
    INTERNAL_BOTTOM_LEFT_X, INTERNAL_BOTTOM_LEFT_Y = 0, 4

    # The offset of the bottom left piece and the spacing of the pieces
    REAL_BOTTOM_LEFT_X, REAL_BOTTOM_LEFT_Y, REAL_SPACING_X, REAL_SPACING_Y = [-0.0555, -0.016, -0.0187, 0.0101] 

    # Getting the position of the AR tag with respect to the base frame
    ar_tag_x = trans.x
    ar_tag_y = trans.y
    ar_tag_z = trans.z

    # We first convert the internal x and y values to a real offset before adding the tag offset and the offset of the bottom left peg
    # The x and y in the board different from the x and y in the base frame
    new_x = (y - INTERNAL_BOTTOM_LEFT_Y) * REAL_SPACING_X + REAL_BOTTOM_LEFT_X + ar_tag_x
    new_y = (x - INTERNAL_BOTTOM_LEFT_X) * REAL_SPACING_Y + REAL_BOTTOM_LEFT_Y + ar_tag_y
    new_z = ar_tag_z

    return [new_x, new_y, new_z]

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    # Set up the right gripper
    return robot_gripper.Gripper('right_gripper', calibrate=True)

def control_gripper(right_gripper, open):
    """
    Control the gripper
    right_gripper: Gripper object
    open: bool determining whether to close or open the gripper
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0
    # Higher values close it up
    # Lower values open it up

    # The open and close values for the gripper
    open_val, close_val = [0.026, 0.034]

    # Open the right gripper
    if open:
        print('Opening gripper.')
        right_gripper.set_position(open_val)
        rospy.sleep(SLEEP)

    # Close the right gripper
    else:
        print('Closing gripper.')
        right_gripper.set_position(close_val)
        rospy.sleep(SLEEP)

def move_robot(position, height_offset):
    """
    Used to move the end effector of the sawyer to a specified position with some height shift in the z direction
    position: [x, y, z]
    height_offset: int
    """
   # Initialize group for the Sawyer arm
    group = MoveGroupCommander("right_arm")

    # Set up planner for precision and shortest path
    group.set_planner_id("PRMstarkConfigDefault") # Optimal Smooth Paths
    group.set_planning_time(3)  # Increase planning time
    group.set_goal_tolerance(0.0001)  # Set the joint, position and orientation goal tolerances simultaneously 
    group.set_num_planning_attempts(5)  # Try multiple times
    group.set_max_velocity_scaling_factor(0.5)  # Slow down for precision
    group.set_max_acceleration_scaling_factor(0.005)  # Reduce jerks
    group.set_workspace([0.4, -0.5, -0.17, 0.9, 0.6, 0.1]) # Setting the workspace 

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

    group.set_pose_target(goal_pose) # Set the pose target
    plan = group.plan() # Plan path
    group.execute(plan[1]) # Execute the plan
    rospy.sleep(SLEEP) # Give the robot some time to settle

def callback(message):
    """
    Main loop
    """
    # Unpack the message    
    start_x, start_y = message.start_x, message.start_y
    end_x, end_y = message.end_x, message.end_y
    rospy.loginfo(f"Message recieved: {start_x}, {start_y}, {end_x}, {end_y}.") # Log the received message

    tuck(ALICE_AR_TUCK) # Tucks the robot into a position where it can see the ar_tag

    right_gripper = calibrate_gripper() # Create the gripper object
    control_gripper(right_gripper, True) # Ensure that the gripper is initially open

    trans = lookup_tag(AR_MARKER) # Get the real world position of the ar tag
    rospy.loginfo(trans) # Log the position of ar tag 
    start_position = convert_internal_coordinates_to_real_coordinates(start_x, start_y, trans) # Get the real world position of the start position
    end_position = convert_internal_coordinates_to_real_coordinates(end_x, end_y, trans) # Get the real world position of the end position
    rospy.loginfo(f"{start_position}, {end_position}") # Log the real coordinates of the start and end position in the base frame

    tuck(REGULAR_TUCK) # Move the robot to a good picking position

    # Pick up the ball at the start position
    move_robot(start_position, TOP_HEIGHT)
    move_robot(start_position, PICKUP_HEIGHT)
    control_gripper(right_gripper, False)
    move_robot(start_position, TOP_HEIGHT)

    tuck(REGULAR_TUCK) # Move the robot to a good picking position

    # Drop the ball at the end position
    move_robot(end_position, TOP_HEIGHT)
    move_robot(end_position, PICKUP_HEIGHT)
    control_gripper(right_gripper, True)
    move_robot(end_position, TOP_HEIGHT)

    # Move the arm to a spot that doesn't block the camera
    tuck(CAMERA_TUCK)

if __name__ == "__main__":
    rospy.init_node('move_board_subscriber') # Create the ros node
    rospy.Subscriber('game_move', BoardMove, callback) # Subscribe to the game_move topic which publishes the BoardMove message
    rospy.spin() # Wait for the callback