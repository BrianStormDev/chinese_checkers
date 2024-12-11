#!/usr/bin/env python
"""
Starter script for 106a lab7. 
Author: Chris Correa
"""

# Old tuck position angles
# 0 -1 0 1.5 0 -0.5 1.7

# New tuck angles
# -0.25 -0.25 0 1.5 0 -1.25 1.7

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
from utils.utils import *

from trac_ik_python.trac_ik import IK

import rospy
import tf2_ros
import intera_interface
from moveit_msgs.msg import DisplayTrajectory, RobotState
from sawyer_pykdl import sawyer_kinematics

# Added dependencies for project
from intera_interface import gripper as robot_gripper
from internal.msg import MoveBoard
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('sawyer_full_stack')
        launch_path = path + '/launch/custom_sawyer_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
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
        target_pos[2] += 0.19 # linear path moves to a Z position above target_pos.
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
    elif task == 'circle':
        target_pos = tag_pos[0]
        target_pos[2] += 0.5
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.1, total_time=15)

    else:
        raise ValueError('task {} not recognized'.format(task))
    
    # target_pos = [0.761, -0.324, -0.02]
    # trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
    
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

def convert_internal_coordinates_to_real_coordinates(x: int, y: int, transform, tag_number) -> PointStamped:
    """
    Takes in internal coordinates of the board and converts them into real world coordinates
    """
    # We need to get the coordinates of the point closest to the ar tag
    # get the rest of the pegs with respect to that point
    # TODO MEASURE THE DISTANCE OF PEGS WITH WRT to the AR TAG
    new_x = x
    new_y = y

    # Do some transform math to get it in respect to the actual robot
    # The yaw should be the thing we are looking at
    # Create a PointStamped in frame y
    point_in_ar = PointStamped()
    point_in_ar.header.frame_id = f"ar_marker_{tag_number}"
    point_in_ar.point.x = new_x
    point_in_ar.point.y = new_y
    point_in_ar.point.z = 0

    # The balls are 6.875 mm in radius, maybe we should shift the z height up?

    # Transform to frame x
    point_in_base = do_transform_point(point_in_ar, transform)
    return point_in_base

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
        right_gripper.close(0.027)

    # Close the right gripper
    else:
        print('Closing gripper.')
        right_gripper.open(0.034)


def callback(message):
    """
    Examples of how to run me:
    python scripts/main.py --help <------This prints out all the help messages
    and describes what each parameter is
    python scripts/main.py -t line -ar_marker 3 -c torque --log
 
    You can also change the rate, timeout if you want
    """

    # task: line, circle.  Default: line
    # ar_marker: Which AR marker to use.  Default: 1
    # controller_name: moveit, open_loop, pid.  Default: moveit
    # rate: This specifies how many ms between loops.  It is important to use a rate
    # and not a regular while loop because you want the loop to refresh at a
    # constant rate, otherwise you would have to tune your PD parameters if 
    # the loop runs slower / faster.  Default: 200
    # timeout: after how many seconds should the controller terminate if it hasn't already. Default: None
    # num_way: How many waypoints for the :obj:`moveit_msgs.msg.RobotTrajectory`.  Default: 300
    # log: plots controller performance.  Default: False

    # Default parameters
    task = 'line'
    ar_marker = 0
    controller_name = 'pid'
    rate = 200
    timeout = None
    num_way = 50
    log=True

    # Unpack the message    
    start_x = message.start_x
    start_y = message.start_y
    end_x = message.end_x
    end_y = message.end_y

    # Tucks the robot
    tuck()

    # Calibrates the gripper and initializes the right gripper object through which the gripper can be controlled
    right_gripper = calibrate_gripper()
    
    # this is used for sending commands (velocity, torque, etc) to the robot
    ik_solver = IK("base", "right_gripper_tip")
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")

    # Convert the internal points to real world points
    transform = lookup_tag(ar_marker)
    start_PointStamped = convert_internal_coordinates_to_real_coordinates(start_x, start_y, transform)
    start_position = np.array([getattr(start_PointStamped.Point, dim) for dim in ('x', 'y', 'z')])

    end_PointStamped = convert_internal_coordinates_to_real_coordinates(end_x, end_y, transform)
    end_position = np.array([getattr(end_PointStamped.Point, dim) for dim in ('x', 'y', 'z')])
    
    # Move the robot to the specified position
    move_robot(task, controller_name, rate, timeout, num_way, log, ik_solver, limb, kin, start_position)
    rospy.sleep(1.0)

    # Close the gripper
    control_gripper(right_gripper, False)
    rospy.sleep(1.0)

    # Figure out how to raise the arm
    tuck()
    rospy.sleep(1.0)

    # Move the arm to the designated position
    move_robot(task, controller_name, rate, timeout, num_way, log, ik_solver, limb, kin, end_position)
    rospy.sleep(1.0)

    # Open the gripper
    control_gripper(right_gripper, True)
    rospy.sleep(1.0)

    # Figure out how to move the arm higher then into the tucked position
    tuck()

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
    if controller_name != "moveit":
        plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])

    if controller_name == "moveit":
        try:
            input('Press <Enter> to execute the trajectory using MOVEIT')
        except KeyboardInterrupt:
            sys.exit()
        # Uses MoveIt! to execute the trajectory.
        planner.execute_plan(robot_trajectory)
    else:
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
    rospy.Subscriber('move_board_topic', MoveBoard, callback)
    rospy.spin()
