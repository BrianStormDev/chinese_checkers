#!/usr/bin/env python
import rospy
import tf2_ros

points = []

def listener():
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try: 
            trans = tfBuffer.lookup_transform("base", "right_gripper_tip", rospy.Time())
            print(trans)
            user = input("Enter y to save, n to stop, p to pass: ")
            if user == "y":
                 points.append(trans.transform.translation)
            elif user == "n":
                 break
            elif user == "p":
                 continue
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("error")
    # Do some path on the points
    # Print the calibrated values

if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)
	listener()