from Image import Image
from rectify import rectify_image
import cv2

# image_path = r"original_images\tape_image.jpg"
# image_path = r"rectified_images\tape_image_rectified.png"
# image_path = r"rectified_images\new_image_7_rectified.png"
image_path = r"original_images\new_image_7.jpg"
image = cv2.imread(image_path)
test_image = Image(image_path, image)

origin_x = int(0.4 * test_image.width)
origin_y = int(0.4 * test_image.height)
width = int(0.3 * test_image.width)
height = int(0.4 * test_image.height)
cropped_image = test_image.crop_image(origin_x, origin_y, width, height)
cv2.imshow("Cropped Image", cropped_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

corners = test_image.find_corners(5)
print(corners)

# top_down_image = rectify_image(image, corners, (500, 500))
top_down_image = test_image.rectify(500, 500)
cv2.imshow("Rectified Image", top_down_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

