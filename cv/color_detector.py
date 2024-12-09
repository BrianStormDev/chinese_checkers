import cv2
import numpy as np

# Read the image
image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\results\image_name.png")
# image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\image_5_rectified.jpg")
# image = cv2.imread(r"C:\Users\gameb\Desktop\chinese_checkers\cv\rectified_images\image_2_rectified.jpg")

# Convert the image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the range of red color in HSV
lower_red = np.array([160, 120, 50])
upper_red = np.array([180, 255, 255])

# Define the range of orange color in HSV
lower_orange = np.array([0, 130, 130])
upper_orange = np.array([20, 255, 255])

# Define the range of yellow color in HSV
lower_yellow = np.array([20, 70, 0])
upper_yellow = np.array([40, 255, 255])

# Define the range of green color in HSV
lower_green = np.array([70, 175, 0])
upper_green = np.array([95, 255, 255])

# # Define the range of blue color in HSV
lower_blue = np.array([100, 180, 0])
upper_blue = np.array([180, 255, 255])

# # Define the range of purple color in HSV
lower_purple = np.array([105, 80, 80])
upper_purple = np.array([140, 200, 255])


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
        if radius > 7:
            circles.append([center, radius])
    
    # Return a list of centers and radii
    return circles

def circle_colors(image, color_name, bgr_tuple, circles):
    # Draw circles around the red points
    for circle in circles:
        cv2.circle(image, circle[0], circle[1], bgr_tuple, 2)
    # Display the result
    cv2.imshow(f'Image with {color_name} Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

red_circles = color_finder(lower_red, upper_red)
orange_circles = color_finder(lower_orange, upper_orange)
yellow_circles = color_finder(lower_yellow, upper_yellow)
green_circles = color_finder(lower_green, upper_green)
blue_circles = color_finder(lower_blue, upper_blue)
purple_circles = color_finder(lower_purple, upper_purple)

red_points = [circle[0] for circle in red_circles]
orange_points = [circle[0] for circle in orange_circles]
yellow_points = [circle[0] for circle in yellow_circles]
green_points = [circle[0] for circle in green_circles]
blue_points = [circle[0] for circle in blue_circles]
purple_points = [circle[0] for circle in purple_circles]

circle_colors(image, "red", (0, 0, 255), red_circles)
circle_colors(image, "orange", (0, 165, 255), orange_circles)
circle_colors(image, "yellow", (0, 255, 255), yellow_circles)
circle_colors(image, "green", (0, 255, 0), green_circles)
circle_colors(image, "blue", (255, 0, 0), blue_circles)
circle_colors(image, "purple", (255, 0, 255), purple_circles)

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

# # We need to have some kind of function that takes a list of points going from top down and return a board from that
# def plot_points(top_down_points):
