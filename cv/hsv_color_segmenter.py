# #!/usr/bin/env python
# # Taken from Lab 8, will help us identify the various colors
# import cv2
# import numpy as np
# # import rospy
# # from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge, CvBridgeError

# image = cv2.imread('cv/image_1.jpg')

# cv2.namedWindow('image')
# cv2.createTrackbar('HMin', 'image', 0, 179, lambda x: None)  # Hue is from 0-179 for OpenCV
# cv2.createTrackbar('SMin', 'image', 0, 255, lambda x: None)
# cv2.createTrackbar('VMin', 'image', 0, 255, lambda x: None)
# cv2.createTrackbar('HMax', 'image', 0, 179, lambda x: None)
# cv2.createTrackbar('SMax', 'image', 0, 255, lambda x: None)
# cv2.createTrackbar('VMax', 'image', 0, 255, lambda x: None)

# # Set default value for MAX HSV trackbars.
# cv2.setTrackbarPos('HMax', 'image', 179)
# cv2.setTrackbarPos('SMax', 'image', 255)
# cv2.setTrackbarPos('VMax', 'image', 255)

# # Initialize variables
# hMin = sMin = vMin = hMax = sMax = vMax = 0
# phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# # Get current positions of all trackbars
# hMin = cv2.getTrackbarPos('HMin', 'image')
# sMin = cv2.getTrackbarPos('SMin', 'image')
# vMin = cv2.getTrackbarPos('VMin', 'image')
# hMax = cv2.getTrackbarPos('HMax', 'image')
# sMax = cv2.getTrackbarPos('SMax', 'image')
# vMax = cv2.getTrackbarPos('VMax', 'image')
# # Set minimum and maximum HSV values for thresholding
# lower = np.array([hMin, sMin, vMin])
# upper = np.array([hMax, sMax, vMax])
# # Convert the image to HSV and apply the threshold
# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# mask = cv2.inRange(hsv, lower, upper)
# output = cv2.bitwise_and(image, image, mask=mask)

# cv2.imshow('thresholded_img', output)

import cv2
import numpy as np

# Load an image
image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\new_image_4_rectified.png")  # Replace with your image path
cv2.namedWindow('image')

# Create trackbars
cv2.createTrackbar('HMin', 'image', 0, 179, lambda x: None)  # Hue is from 0-179 in OpenCV
cv2.createTrackbar('SMin', 'image', 0, 255, lambda x: None)
cv2.createTrackbar('VMin', 'image', 0, 255, lambda x: None)
cv2.createTrackbar('HMax', 'image', 179, 179, lambda x: None)
cv2.createTrackbar('SMax', 'image', 255, 255, lambda x: None)
cv2.createTrackbar('VMax', 'image', 255, 255, lambda x: None)

while True:
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')
    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    # Define the HSV range for the mask
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # lower = np.array([80, 100, 0])
    # upper = np.array([150, 80, 210])

    # Convert image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply the mask
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    # Display the result
    cv2.imshow('image', output)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

# Lower Bound: (80, 0, 100)
# Upper Bound: (150, 80, 210)