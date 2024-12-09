import sys
import cv2
from rectify import rectify_image
from point_reader import scale_points
from point_picker import pick_points
import numpy as np

def main():
    if (len(sys.argv) != 5):
        print("Usage: python <script_name.py> <out_name> <img1_path> <width> <height>")
        return 

    out_name = sys.argv[1]
    img_path = sys.argv[2]
    width = int(sys.argv[3])
    height = int(sys.argv[4])
    points = np.array(pick_points(img_path))

    img = cv2.imread(img_path)
    img_dim = (img.shape[1], img.shape[0])
    img_pts = scale_points(points, img_dim)
    rectified_dim = (width, height)
    rectified_img = rectify_image(img, img_pts, rectified_dim)
    cv2.imwrite("results/" + out_name + ".png", rectified_img)

if __name__ == "__main__":
    main()