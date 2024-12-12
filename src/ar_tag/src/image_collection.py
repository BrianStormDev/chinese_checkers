#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os

class CameraDisplayNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_display_node', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        # Replace '/camera/image_raw' with the correct topic for your camera
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

        self.image = None

        # Create a directory to save images if it doesn't exist
        self.save_directory = rospy.get_param('~save_directory', 'new_images/')
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)
        else: 
            print(self.save_directory)

    def callback(self, msg):
        try:
            # Convert the ROS Image message to a OpenCV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image = cv_image

            # Display the image using OpenCV
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)  # Update the image window

        except Exception as e:
            rospy.logerr("Error in callback: %s", str(e))

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an object of the CameraDisplayNode class and start the node
        node = CameraDisplayNode()
        node.run()
        print('Node running')
    except rospy.ROSInterruptException:
        pass
    finally:
        # # Close any OpenCV windows when the node is shut down
        if node.image is not None: 
            print(node.save_directory)
            filename = os.path.join(node.save_directory, f"new_image_{27}.jpg")
            cv2.imwrite(filename, node.image)
            print('File saved')
        else: 
            print('File not saved')
        cv2.destroyAllWindows()
