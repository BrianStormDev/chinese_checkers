import cv2
import numpy as np

# Load the image
image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\image_5_rectified.jpg")

x_dim = 26
y_dim = 18

x_origin = 160
x_final = 1030
y_origin = 60
y_final = 650
x_step = (x_final - x_origin) / x_dim
y_step = (y_final - y_origin) / y_dim

for x in range(x_dim + 1):
    for y in range(y_dim + 1):
        cv2.circle(image, (int(x_origin + x * x_step), int(y_origin + y * y_step)), 5, (0, 255, 0), -1)
    
cv2.imshow('Image', image)
# wait for a key to be pressed to exit 
cv2.waitKey(0) 

# close the window 
cv2.destroyAllWindows() 
