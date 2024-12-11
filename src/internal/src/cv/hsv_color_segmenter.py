import cv2
import numpy as np

# Load an image
image = cv2.imread(r"/home/cc/ee106a/fa24/class/ee106a-air/ros_workspaces/chinese_checkers/src/ar_tag/src/saved_images/new_image_21.jpg")  # Replace with your image path
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