#!/usr/bin/env python
import rospy
# import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander

def move_robot(position, height_offset):
   # Initialize move_group for the Sawyer arm
    group = MoveGroupCommander("right_arm")

    # Set up planner for precision and shortest path
    group.set_planner_id("PRMstarConfigDefault ") # Optimal Smooth Paths
    group.set_planning_time(10)  # Increase planning time for more complex paths
    group.set_goal_tolerance(0.001)  # Set the joint, position and orientation goal tolerances simultaneously 
    group.set_num_planning_attempts(5)  # Try multiple times
    group.set_max_velocity_scaling_factor(0.1)  # Slow down for precision
    group.set_max_acceleration_scaling_factor(0.05)  # Reduce jerks

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

    # Plan Cartesian path for smooth, straight-line movement
    waypoints = [goal_pose.pose]
    (trajectory, fraction) = group.compute_cartesian_path(
        waypoints,
        eef_step=0.001,  # Finer step size for higher precision
        jump_threshold=0.0  # Avoid large, unpredictable jumps
    )

    # Fraction indicates the success rate of the planned trajectory
    if fraction == 1.0:
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")

        # Execute if safe
        if user_input == 'y':
            rospy.loginfo("Executing high-precision trajectory...")
            group.execute(trajectory, wait=True)

if __name__ == "__main__":
    move_robot()
