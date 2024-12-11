from sensor_msgs.msg import Image
from Image import Image as I
import cv2, rospy
from cv_bridge import CvBridge


def call_back(raw_image): 
    """
    Input: image
    Output: peg color array
    """

    # Convert the ROS Image message to a OpenCV2 image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(raw_image, "bgr8")
    image = I(None, raw_image)
    h, w, _ = raw_image.shape
    cropped = image.crop_image(int(0.4 * w), int(0.3 * h), int(0.25 * w), int(0.4 * h)) # TODO: fill in parameters
    cv2.imshow('Cropped', cropped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    image.find_corners(50) # TODO: adjust in param
    rectified_image = image.rectify(500, 500)
    cv2.imshow('Corrected', rectified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    exit()

if __name__ == '__main__': 
    rospy.init_node('cali_cropped', anonymous=True)
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, call_back)
    rospy.spin()


