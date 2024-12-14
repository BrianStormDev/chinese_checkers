#!/usr/bin/env python
import rospy
import tf2_ros

# Tuck dependencies
from intera_core_msgs.msg import HeadPanCommand
from intera_interface import Limb

def ar_tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to ar_tuck the arm? (y/n): ') == 'y':
        # Set the head pan
        target_pan = 0
        speed_ratio = 1
        pan_mode = 1

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
        return trans.transform.translation
    except Exception as e:
        print(e)
        return None

points = []

def point_picker():
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try: 
            trans = tfBuffer.lookup_transform("base", "right_gripper_tip", rospy.Time())
            print(trans)
            user = input("Enter y to save, n to stop, p to pass: ")
            if user == "y":
                 points.append(trans.transform.translation)
                 if len(points) == 3:
                     break
            elif user == "n":
                 break
            elif user == "p":
                 continue
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")

def process_points():
    ar_tag = points[0]
    bottom_left = points[1]
    top_right = points[2]

    precision = 4
    x_offset = round(bottom_left.x - ar_tag.x, precision)
    y_offset = round(bottom_left.y - ar_tag.y, precision)
    x_spacing = round((top_right.x - bottom_left.x) / 8, precision)
    y_spacing = round((top_right.y - bottom_left.y) / 24, precision)
    print(f"x_offset: {x_offset}, y_offset: {y_offset}, x_spacing: {x_spacing}, y_spacing: {y_spacing}")
    print([x_offset, y_offset, x_spacing, y_spacing])

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    ar_tuck()
    points.append(lookup_tag(0))
    point_picker()
    process_points()