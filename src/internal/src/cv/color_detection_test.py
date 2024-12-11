from Image import Image
import cv2

origin_image_path = r"src\internal\src\cv\rectified_images\new_image_7_rectified.png"
image = cv2.imread(origin_image_path)
test_image = Image(origin_image_path, image, False, True)

test_image.find_colored_points(50)
test_image.sort_points()
print(test_image.point_colors())