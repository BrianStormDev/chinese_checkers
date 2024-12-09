import cv2
import numpy as np

# Load an image
image = cv2.imread('cv/image_5.jpg')  # Replace with your image path

def crop_image(origin_x, origin_y, height, width):
    crop = image[origin_y: origin_y + height, origin_x: origin_x + width]
    return crop 

# Display the cropped image
height, width, channels, = image.shape
y = int((1/5) * height)
x = int((1/5) * width)
h = int((2/3) * height)
w = int((2/3) * width)

cv2.imshow('Image', crop_image(x, y, h, w))
cv2.waitKey(0)