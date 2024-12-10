from Image import Image
import cv2

origin_image_path = r"rectified_images\new_image_7_rectified.png"
image = cv2.imread(origin_image_path)
test_image = Image(origin_image_path, image, False, True)

test_image.find_colored_points(5)
