from Image import Image
from rectify import rectify_image
import cv2

def point_sort_test():
    points = [(1, 2), (7, 3), (0, 1), (4, 2), (8, 3), (9, 11)]
    y_sorted = sorted(points, key = lambda point: point[1])
    print(y_sorted)
    row_lengths = [1, 2, 3]
    point_index = 0
    sorted_points = []
    for row_length in row_lengths:
        x_sorted = sorted(y_sorted[point_index: point_index + row_length], key = lambda point: point[0])
        sorted_points += x_sorted
        point_index += row_length
    print(sorted_points)
    print(sum([1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]))

def corner_detection_test(origin_image_path, origin_x_frac, origin_y_frac, width_frac, height_frac, corner_threshold):
    """
    origin_image_path: A path to the original image
    origin_x_frac: Fraction of the width where we want to start the crop
    origin_y_frac: Fraction of the height where we want to start the crop
    width_frac: Fraction of the width that we want as the new width of the crop
    height_frac: Fraction of the height that we want as the new height of the crop
    corner_threshold: A minimum area a contour must have to be classified as a corner
    """
    image = cv2.imread(origin_image_path)
    test_image = Image(origin_image_path, image)

    # Step 1: Crop the Image
    origin_x = int(origin_x_frac * test_image.width)
    origin_y = int(origin_y_frac * test_image.height)
    width = int(width_frac * test_image.width)
    height = int(height_frac * test_image.height)
    cropped_image = test_image.crop_image(origin_x, origin_y, width, height)

    # Step 2: Get the corners of the image and rectify it
    corners = test_image.find_corners(corner_threshold)
    print(corners)

    # top_down_image = rectify_image(image, corners, (500, 500))
    top_down_image = test_image.rectify(500, 500)
    cv2.imshow("Rectified Image", top_down_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def color_detection_test(origin_image_path, area_threshold):
    """
    origin_image_path: A path to the original image
    area_threshold: A minimum area a contour must have to be classified as a point of interest
    """
    image = cv2.imread(origin_image_path)
    test_image = Image(origin_image_path, image, False, True)

    test_image.find_colored_points(area_threshold)
    test_image.sort_points()
    print(test_image.point_colors())

def process_image(origin_image_path, origin_x_frac, origin_y_frac, width_frac, height_frac, corner_threshold, area_threshold):
    """
    origin_image_path: A path to the original image
    origin_x_frac: Fraction of the width where we want to start the crop
    origin_y_frac: Fraction of the height where we want to start the crop
    width_frac: Fraction of the width that we want as the new width of the crop
    height_frac: Fraction of the height that we want as the new height of the crop
    area_threshold: A minimum area a contour must have to be classified as a corner
    corner_threshold: A minimum area a contour must have to be classified as a corner
    """
    image = cv2.imread(origin_image_path)
    test_image = Image(origin_image_path, image, False, False)

    # Step 1: Crop the image
    origin_x = int(origin_x_frac * test_image.width)
    origin_y = int(origin_y_frac * test_image.height)
    crop_width = int(width_frac * test_image.width)
    crop_height = int(height_frac * test_image.height)
    cropped_image = test_image.crop_image(origin_x, origin_y, crop_width, crop_height)
    cv2.imshow("Cropped Image", cropped_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 
    
    # Step 2: Find the corners 
    print(test_image.find_corners(corner_threshold))

    # Step 3: Rectify the image
    rectified_image = test_image.rectify(500, 500)

    cv2.imshow("Rectified Image", rectified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows() 

    # Step 4: Find the colored points 
    test_image.find_colored_points(area_threshold)

if __name__ == "__main__":
    test_image_path = r"original_images\new_image_22.jpg"
    # corner_detection_test(test_image_path, 0.4, 0.3, 0.4, 0.4, 5)
    # color_detection_test(test_image_path, 50)
    process_image(test_image_path, 0.3, 0.1, 0.4, 0.5, 10, 10)
