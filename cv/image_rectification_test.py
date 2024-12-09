import os
from os import listdir
import cv2
from image_rectifier import get_top_down_image

IMAGE_WIDTH = 500
IMAGE_HEIGHT = 500
image_directory = r"C:\Users\gameb\Desktop\chinese_checkers\cv\original_images"
target_directory = r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images"

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
    full_image_names = get_directory_image_names(origin_directory)
    for full_image_name in full_image_names:
        top_down_image = get_top_down_image(image_directory + "\\" + full_image_name, IMAGE_WIDTH, IMAGE_HEIGHT)
        # We slice away the last four elements because those are the .png and .jpg elements
        image_ending = full_image_name[-4:]
        image_name = full_image_name[:-4]
        new_image_name = image_name + "_rectified" + image_ending
        new_image_path = target_directory + "\\" + new_image_name
        cv2.imwrite(new_image_path, top_down_image)

if __name__ == "__main__":
    rectify_images(image_directory, target_directory)
