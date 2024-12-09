# import cv2
# import numpy as np

# # Load the image
# image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\image_1_rectified.jpg")
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# # Threshold the image to find dark regions
# _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

# cv2.imshow('Grayscale Image', gray)

# # Find contours
# contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# # Loop through contours to find rectangles
# for contour in contours:
#     # Approximate the contour to a polygon
#     epsilon = 0.02 * cv2.arcLength(contour, True)
#     approx = cv2.approxPolyDP(contour, epsilon, True)

#     # Check if the polygon has 4 vertices
#     if len(approx) == 4:
#         # Get bounding box and draw it
#         x, y, w, h = cv2.boundingRect(approx)
#         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

# # Display the result
# cv2.imshow("Detected Black Boxes", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
import numpy as np

# Load the image
image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\image_1_rectified.jpg")

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow("Grey", gray_image)

# Threshold the grayscale image to find black pixels
# Any pixel value less than a certain threshold (e.g., 50) will be considered black
_, black_pixels = cv2.threshold(gray_image, 120, 255, cv2.THRESH_BINARY_INV)

# Display the black pixels (black areas will be white in this image)
cv2.imshow('Black Pixels', black_pixels)

# Wait for a key press and close all windows
cv2.waitKey(0)
cv2.destroyAllWindows()

# Hmmm, we can wait for the ar tag detection network