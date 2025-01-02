#!/usr/bin/env python
# Used to calibrate the parameters of the inference function which is used to calculate the position of the pieces
# Grabs the real world positions of a list of points and then computes the least squares solution to get the parameters
import rospy
import tf2_ros
import numpy as np

# Tuck dependencies
from intera_core_msgs.msg import HeadPanCommand
from intera_interface import Limb

# How long to sleep in between actions
SLEEP = 1.0

def tuck(angles):
    """
    Tuck the robot arm to a custom position
    angles: list of angles (floats)
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
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

AR_TUCK = [0, -0.5, 0, 1.25, 0, -0.75, 1.7]
TAG_NUMBER = 0 
INTERNAL_COORDS = [[0, 4], [0, 12], [12, 16], [24, 12], [24, 4], [12, 0], [12, 8]]
NUM_POINTS = len(INTERNAL_COORDS)
POINTS = []
PRECISION = 4
INTERNAL_Y, INTERNAL_X = 4, 0

def point_picker():
    """
    Allows the user to move the end effector to a spot of their choosing and 
    then add that real world coordinate to the list of points
    Entering y saves a point to the list and entering p allows for the position to update
    """
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    
    while not rospy.is_shutdown():
        try: 
            # Grabs the transformation between the base of the arm and its gripper
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            trans = tfBuffer.lookup_transform("base", "right_gripper_tip", rospy.Time(0), rospy.Duration(10.0))
            user = input("Enter y to save, p to pass: ")
            if user == "y":
                 POINTS.append(trans.transform.translation)
                 if len(POINTS) == NUM_POINTS:
                     break
            elif user == "p":
                 continue
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")

def process_points(ar_tag):
    """
    Processes the points and solves for the least squares solution
    ar_tag: geometry_msgs/Vector3 of the ar_tag relative to the base frame
    """
    A, B = [], []

    for i in range(NUM_POINTS):
        internal_coord = INTERNAL_COORDS[i]
        point = POINTS[i]
        
        A.append([1, 0, internal_coord[1] - INTERNAL_Y, 0])
        B.append([point.x - ar_tag.x])
        
        A.append([0, 1, 0, internal_coord[0] - INTERNAL_X])
        B.append([point.y - ar_tag.y])
    
    # Convert A and B into np.arrays and get the least squares solution
    A, B = np.array(A), np.array(B)
    x, _, _, _ = np.linalg.lstsq(A, B, rcond = None) # rcond used to supress a warning
    
    # Print the optimal parameters
    optimal_parameters = [x[j][0] for j in range(4)]
    print(optimal_parameters)
    
    # Print the optimal parameters rounded
    optimal_parameters_rounded = [round(x[j][0], PRECISION) for j in range(4)]
    print(optimal_parameters_rounded)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True) # Create the ros node
    
    # Tuck the arm and get the position of the ar_tag
    tuck(AR_TUCK)
    ar_tag_pos = lookup_tag(TAG_NUMBER)
    
    point_picker() # Get the points of the pieces you want to calibrate
    process_points(ar_tag_pos) # Process the points