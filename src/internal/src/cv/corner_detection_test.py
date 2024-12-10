from Image import Image
from rectify import rectify_image
import cv2

# image_path = r"original_images\tape_image.jpg"
image_path = r"rectified_images\tape_image_rectified.png"
# image_path = r"rectified_images\new_image_7_rectified.png"
image = cv2.imread(image_path)
test_image = Image(image_path, image)

corners = test_image.find_corners()
print(corners)

top_down_image = rectify_image(image, corners, (500, 500))
cv2.imshow("homographied", top_down_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

