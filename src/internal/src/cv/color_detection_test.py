from Image import Image
import cv2

image_path = r"rectified_images\new_image_7_rectified.png"
image = cv2.imread(image_path)
test_image = Image(image_path, image)

test_image.find_colored_points()
