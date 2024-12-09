import os
from os import listdir
import cv2
import numpy as np

IMAGE_WIDTH = 1200
IMAGE_HEIGHT = 700
image_directory = r"C:\Users\gameb\Desktop\chinese_checkers\cv\original_images"
target_directory = r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images"

points_1 = [[(575, 178), (907, 208), (919, 508), (501, 452)]]
points_2 = [[(490, 205), (828, 208), (877, 508), (447, 501)]]
points_3 = [[(489, 203), (829, 206), (877, 509), (447, 501)]]
points_4 = [[(490, 203), (830, 205), (875, 509), (447, 504)]]
points_5 = [[(497, 194), (833, 200), (879, 499), (451, 488)]]
points_6 = [[(501, 194), (838, 202), (879, 503), (451, 487)]]
points = points_1 + points_2 + points_3 + points_4 + points_5 + points_6

def get_directory_image_names(folder_dir):
    image_names = []
    for image in os.listdir(folder_dir):
        # Check if the image ends with png or jpg
        if (image.endswith(".png") or image.endswith(".jpg")):
            image_names.append(image)
    return image_names

def rectify_images(origin_directory, target_directory):
    """
    This function rectifies the images and places them in the target_directory
    """
    # Define destination points (desired rectangle for rectification)
    dst_points = np.array([
        [0, 0],         # Top-left
        [IMAGE_WIDTH-1, 0],   # Top-right
        [IMAGE_WIDTH-1, IMAGE_HEIGHT-1],  # Bottom-right
        [0, IMAGE_HEIGHT-1]   # Bottom-left
    ], dtype=np.float32)

    full_image_names = get_directory_image_names(origin_directory)
    for i in range(6):
        src_points = np.array(points[i])
        H, _ = cv2.findHomography(src_points, dst_points)
        full_image_name = full_image_names[i]
        image_path = image_directory + "\\" + full_image_name
        image = cv2.imread(image_path)
        top_down_image = cv2.warpPerspective(image, H, (IMAGE_WIDTH, IMAGE_HEIGHT))
        # We slice away the last four elements because those are the .png and .jpg elements
        image_ending = full_image_name[-4:]
        image_name = full_image_name[:-4]
        new_image_name = image_name + "_rectified" + image_ending
        new_image_path = target_directory + "\\" + new_image_name
        cv2.imwrite(new_image_path, top_down_image)

if __name__ == "__main__":
    rectify_images(image_directory, target_directory)

