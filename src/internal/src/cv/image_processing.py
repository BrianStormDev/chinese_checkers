import cv2
import numpy as np

def process_image(image):
    """
    image: An image matrix
    """
    # Step 1: Crop the image
    cropped_image = crop_image(image)

    # Step 2: Detect the corners on the image
    
    cv2.imshow('image', cropped_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

process_image(cv2.imread(r"original_images/new_image_7.jpg"))