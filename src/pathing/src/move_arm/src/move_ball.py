#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from intera_interface import gripper as robot_gripper
import sys

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "stp_022312TP99620_tip_1"
        #SHOULD BE right_gripper_tip for anything other than station 

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.651
        request.ik_request.pose_stamped.pose.position.y = -0.101
        request.ik_request.pose_stamped.pose.position.z = -0.120      
        request.ik_request.pose_stamped.pose.orientation.x = 0
        request.ik_request.pose_stamped.pose.orientation.y = 1
        request.ik_request.pose_stamped.pose.orientation.z = 0
        request.ik_request.pose_stamped.pose.orientation.w = 0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                right_gripper.open()
                group.execute(plan[1])
                right_gripper.close()
                rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.643
        request.ik_request.pose_stamped.pose.position.y = -0.089
        request.ik_request.pose_stamped.pose.position.z = 0.0388     
        request.ik_request.pose_stamped.pose.orientation.x = 0
        request.ik_request.pose_stamped.pose.orientation.y = 1
        request.ik_request.pose_stamped.pose.orientation.z = 0
        request.ik_request.pose_stamped.pose.orientation.w = 0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
                right_gripper.open()
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()

# #!/usr/bin/env python
# import rospy
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
# from geometry_msgs.msg import PoseStamped
# from moveit_commander import MoveGroupCommander
# import numpy as np
# from numpy import linalg
# from intera_interface import gripper as robot_gripper
# import sys

# def main():
#     # Wait for the IK service to become available
#     rospy.wait_for_service('compute_ik')
#     rospy.init_node('service_query')

#     # Set up the right gripper
#     right_gripper = robot_gripper.Gripper('right_gripper')

#     # Create the function used to call the service
#     compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
#     while not rospy.is_shutdown():
#         input('Press [ Enter ]: ')
        
#         # Construct the request
#         request = GetPositionIKRequest()
#         request.ik_request.group_name = "right_arm"

#         # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
#         link = "stp_022312TP99620_tip_1"

#         request.ik_request.ik_link_name = link
#         # request.ik_request.attempts = 20
#         request.ik_request.pose_stamped.header.frame_id = "base"
        
#         # Set the desired orientation for the end effector HERE
#         request.ik_request.pose_stamped.pose.position.x = 0.715
#         request.ik_request.pose_stamped.pose.position.y = 0.009
#         request.ik_request.pose_stamped.pose.position.z = -0.120      
#         request.ik_request.pose_stamped.pose.orientation.x = 0
#         request.ik_request.pose_stamped.pose.orientation.y = 1
#         request.ik_request.pose_stamped.pose.orientation.z = 0
#         request.ik_request.pose_stamped.pose.orientation.w = 0
        
#         try:
#             # Send the request to the service
#             response = compute_ik(request)
            
#             # Print the response HERE
#             print(response)
#             group = MoveGroupCommander("right_arm")

#             # Setting position and orientation target
#             group.set_pose_target(request.ik_request.pose_stamped)

#             # Plan IK
#             plan = group.plan()
#             user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
#             # Execute IK if safe
#             if user_input == 'y':
#                 right_gripper.open()
#                 group.execute(plan[1])
#                 right_gripper.close()
#                 rospy.sleep(1.0)
            
#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)


#         input('Press [ Enter ]: ')
        
#         # Construct the request
#         request = GetPositionIKRequest()
#         request.ik_request.group_name = "right_arm"

#         # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
#         link = "right_gripper_tip"

#         request.ik_request.ik_link_name = link
#         # request.ik_request.attempts = 20
#         request.ik_request.pose_stamped.header.frame_id = "base"
        
#         # Set the desired orientation for the end effector HERE
#         request.ik_request.pose_stamped.pose.position.x = 0.685
#         request.ik_request.pose_stamped.pose.position.y = -0.326
#         request.ik_request.pose_stamped.pose.position.z = -0.108     
#         request.ik_request.pose_stamped.pose.orientation.x = 0
#         request.ik_request.pose_stamped.pose.orientation.y = 1
#         request.ik_request.pose_stamped.pose.orientation.z = 0
#         request.ik_request.pose_stamped.pose.orientation.w = 0
        
#         try:
#             # Send the request to the service
#             response = compute_ik(request)
            
#             # Print the response HERE
#             print(response)
#             group = MoveGroupCommander("right_arm")

#             # Setting position and orientation target
#             group.set_pose_target(request.ik_request.pose_stamped)

#             # Plan IK
#             plan = group.plan()
#             user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
#             # Execute IK if safe
#             if user_input == 'y':
#                 group.execute(plan[1])
#                 right_gripper.open()
            
#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)

# # Python's syntax for a main() method
# if __name__ == '__main__':
#     main()

