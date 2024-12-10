import cv2
import numpy as np

# Read the image
# image = cv2.imread(r"results\image_name.png")
# image = cv2.imread(r"rectified_images\image_5_rectified.jpg")
# image = cv2.imread(r"rectified_images\image_2_rectified.jpg")
# image = cv2.imread(r"rectified_images\new_image_1_rectified.png")
# image = cv2.imread(r"rectified_images\new_image_2_rectified.png")
# image = cv2.imread(r"rectified_images\new_image_4_rectified.png")
# image = cv2.imread(r"rectified_images\new_image_7_rectified.png")
image = cv2.imread(r"original_images\new_image_7.jpg")

height, width, channels = image.shape

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# # Define the range of blue tape color in HSV
lower_blue = np.array([100, 120, 0])
upper_blue = np.array([110, 255, 255])


def color_finder(lower_hsv, upper_hsv):
    """
    image: In the HSV Color Space
    """
    circles = []

    # Create a mask for red color
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center = (int(x), int(y))
        radius = int(radius)
        # We want to filter out the noisy points as well as the AR Tag points
        if radius > 7 and not(x <= (1/8 * width) and y > (7/8 * height)):
            circles.append([center, radius])
    
    # Return a list of centers and radii
    return circles

blue_circles = color_finder(lower_blue, upper_blue)

blue_points = [circle[0] for circle in blue_circles]

# circle_colors(image, "blue", (255, 0, 0), blue_circles)

# Now pressuming that we are mapping out the board, we have to do some stuff
# Specifically, we want to sort the y coordinates and based on the x positions, we can figure which one of them comes first
# But it's tricky because it's like we have to find y ranges where the rows are 

# Not really, we can sort by the y coordinate. Then for each rows (1 peg, 3, pegs, etc...) we can sort those slices by x coordinate

def sort_points(points):
    """
    points: An array of point tuples
    """
    # row_lengths = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]
    row_lengths = [4, 3, 2, 1]
    assert len(points) == sum(row_lengths), f"Not enough points detected! Expected {sum(row_lengths)} but got {len(points)}"

    sorted_points = []
    # Sort the points according to their y coordinate
    y_sorted = sorted(points, key = lambda point: point[1])
    # Note that we sort it based on the idea that the zero coordinate for y is at the top
    
    point_index = 0
    # For each of the rows, sort the points by their x coordinate
    for row_length in row_lengths:
        x_sorted = sorted(y_sorted[point_index: row_lengths + 1], key = lambda point: point[0])
        sorted_points += x_sorted
        point_index += row_length
    return sorted_points

# After we sort the points, we just grab the corner elements
def get_corners(points):
    up_left = points[0]
    up_right = points[1]
    down_right = points[-1]
    down_left = (up_left[0], down_right[1])
    return [up_left, up_right, down_left, down_right]

print(get_corners(blue_points))

def plot_corners(image, corners, color_name):
    for corner in corners:
        cv2.circle(image, (corner[0], corner[1]), 2, (0, 255, 0), 2)
    cv2.imshow(f'Image with {color_name} Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

plot_corners(image, get_corners(blue_points), "blue")
