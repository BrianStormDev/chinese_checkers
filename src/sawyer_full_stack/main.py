#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""

# Old tuck position angles
# 0 -1 0 1.5 0 -0.5 1.7

# New tuck angles
# -0.25 -0.5 0 1.5 0 -1.0 1.7

import sys
import numpy as np
import rospkg
import roslaunch

from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from paths.path_planner import PathPlanner
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
import rospy.logger_level_service_caller
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

# Added dependencies for project
from intera_interface import gripper as robot_gripper
from internal.msg import BoardMove
from intera_core_msgs.msg import HeadPanCommand
from intera_interface import Limb

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
        #     'right_j0': -0.25,
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
            'right_j3': 1.5,
            'right_j4': 0,
            'right_j5': -0.85,
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

def get_trajectory(limb, kin, ik_solver, tag_pos, num_way, task):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    if task == 'line':
        target_pos = tag_pos[0]
        # linear path moves to a Z position above target_pos.
        target_pos[2] += 0.205 
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=30)
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller

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
    BOTTOM_LEFT_REAL_X = - 0.05075
    BOTTOM_LEFT_REAL_Y = - 0.01975

    # X - direction
    # Difference of outer edges: 27.0 mm
    # Difference of inner edges: 11.5 mm
    # Radii of holes: 4.5 mm
    # Difference of centers: 27.0 - 4.5 - 4.5 = 18.0 -> 0.180

    # Y - direction
    # Since every x peg is 2 pegs along we divid the gap by 2
    # Difference of outer edges: 29.5 mm
    # Difference of inner edges: 11.5 mm
    # Radii of holes: 4.5 mm
    # Difference of centers: 4.5 + 11.5 +4.5 = 20.5 -> 0.205 -> 0.1025

    REAL_SPACING_X = 0.180
    REAL_SPACING_Y = 0.1025

    # The balls are 6.875 mm in radius, maybe we should shift the z height up?
    HEIGHT_SHIFT = 0
    
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
    new_x = -(y - BOTTOM_LEFT_INTERNAL_Y) * REAL_SPACING_X + BOTTOM_LEFT_REAL_X + ar_tag_x
    new_y = (x - BOTTOM_LEFT_INTERNAL_X) * REAL_SPACING_Y + BOTTOM_LEFT_REAL_Y + ar_tag_y
    new_z = ar_tag_z + HEIGHT_SHIFT

    # # Incase the precision is the issue
    # precision = 2
    # new_x = round(new_x, precision)
    # new_y = round(new_y, precision)
    # new_z = round(new_z, precision)

    return [np.array([new_x, new_y, new_z])]

def calibrate_gripper():
    """
    calibrates the gripper and returns the right_gripper object through which the gripper can be controlled
    """
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    return right_gripper

def control_gripper(right_gripper, open):
    """
    
    """
    # MAX_POSITION = 0.041667 
    # MIN_POSITION = 0.0

    # Open the right gripper
    if open:
        print('Opening gripper.')
        right_gripper.open(0.027)

    # Close the right gripper
    else:
        print('Closing gripper.')
        right_gripper.open(0.033)


def callback(message):
    """
    task: line, circle.  Default: line
    ar_marker: Which AR marker to use.  Default: 1
    controller_name: moveit, open_loop, pid.  Default: moveit
    rate: This specifies how many ms between loops.  It is important to use a rate
    and not a regular while loop because you want the loop to refresh at a
    constant rate, otherwise you would have to tune your PD parameters if 
    the loop runs slower / faster.  Default: 200
    timeout: after how many seconds should the controller terminate if it hasn't already. Default: None
    num_way: How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300
    log: plots controller performance.  Default: False
    """

    # Default parameters
    task = 'line'
    ar_marker = 0
    controller_name = 'pid'
    rate = 200
    timeout = None
    num_way = 1000
    log = False

    # Unpack the message    
    start_x = message.start_x
    start_y = message.start_y
    end_x = message.end_x
    end_y = message.end_y

    rospy.logerr(f"Message recieved: {start_x}, {start_y}, {end_x}, {end_y}.")

    # Tucks the robot into a position where it can see the ar_tag
    ar_tuck()

    # Calibrates the gripper and initializes the right gripper object through which the gripper can be controlled
    right_gripper = calibrate_gripper()
    rospy.sleep(3.0)
    
    # Ensure that the gripper is initially open
    control_gripper(right_gripper, True)
    rospy.sleep(1.0)
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # Convert the internal points to real world points
    transform = lookup_tag(ar_marker)

    # Test by moving to the AR_TAG
    start_position = [np.array([getattr(transform.transform.translation, dim) for dim in ('x', 'y', 'z')])]

    # start_position = convert_internal_coordinates_to_real_coordinates(start_x, start_y, transform)

    end_position = convert_internal_coordinates_to_real_coordinates(end_x, end_y, transform)
    
    rospy.logerr(f"{start_position[0]}, {end_position[0]}")

    # Move the robot to the specified position
    move_robot(task, controller_name, rate, timeout, num_way, log, ik_solver, limb, kin, start_position)
    rospy.sleep(1.0)

    # Close the gripper
    control_gripper(right_gripper, False)
    rospy.sleep(1.0)

    # Move the robot to a good picking position
    regular_tuck()
    rospy.sleep(1.0)

    # Move the arm to the designated position
    move_robot(task, controller_name, rate, timeout, num_way, log, ik_solver, limb, kin, end_position)
    rospy.sleep(1.0)

    # Open the gripper
    control_gripper(right_gripper, True)
    rospy.sleep(1.0)

    # Move the arm to a spot that doesn't block the camera
    camera_tuck()
    rospy.sleep(1.0)

def move_robot(task, controller_name, rate, timeout, num_way, log, ik_solver, limb, kin, position):
    """
    Moves the robot to a specified location
    """
    # Get an appropriate RobotTrajectory for the task (circular, linear, or square)
    # If the controller is a workspace controller, this should return a trajectory where the
    # positions and velocities are workspace positions and velocities.  If the controller
    # is a jointspace or torque controller, it should return a trajectory where the positions
    # and velocities are the positions and velocities of each joint.
    robot_trajectory = get_trajectory(limb, kin, ik_solver, position, num_way, task)

    # This is a wrapper around MoveIt! for you to use.  We use MoveIt! to go to the start position
    # of the trajectory
    planner = PathPlanner('right_arm')
    
    # By publishing the trajectory to the move_group/display_planned_path topic, you should 
    # be able to view it in RViz.  You will have to click the "loop animation" setting in 
    # the planned path section of MoveIt! in the menu on the left side of the screen.
    pub = rospy.Publisher('move_group/display_planned_path', DisplayTrajectory, queue_size=10)
    disp_traj = DisplayTrajectory()
    disp_traj.trajectory.append(robot_trajectory)
    disp_traj.trajectory_start = RobotState()
    pub.publish(disp_traj)

    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    controller = get_controller(controller_name, limb, kin)
    try:
        input('Press <Enter> to execute the trajectory using YOUR OWN controller')
    except KeyboardInterrupt:
        sys.exit()
    # execute the path using your own controller.
    done = controller.execute_path(
        robot_trajectory, 
        rate=rate, 
        timeout=timeout, 
        log=log
    )
    if not done:
        print('Failed to move to position')
        sys.exit(0)

if __name__ == "__main__":
    rospy.init_node('move_board_subscriber')
    rospy.Subscriber('game_move', BoardMove, callback)
    rospy.spin()
