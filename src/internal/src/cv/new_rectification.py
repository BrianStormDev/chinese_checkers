import sys
import cv2
from rectify import rectify_image
from point_reader import scale_points
from point_picker import pick_points
import numpy as np

def main():

    i = 16

    img_path = f'../../../ar_tag/src/saved_images/new_image_{i + 9}.jpg'
    width = 500
    height = 500
    points = np.array(pick_points(img_path))

    img = cv2.imread(img_path)
    img_dim = (img.shape[1], img.shape[0])
    img_pts = scale_points(points, img_dim)
    rectified_dim = (width, height)
    rectified_img = rectify_image(img, img_pts, rectified_dim)
    cv2.imwrite(f"new_rectified_images/new_data/image_{i}.jpg", rectified_img)
    cv2.imshow("window", rectified_img)
    cv2.waitKey()

if __name__ == "__main__":
    main()