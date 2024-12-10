from Color import Color
import numpy as np
import cv2
from rectify import rectify_image
from point_reader import read_points
from warp_img import compute_homography, warp_image

class Image:
    red = Color("red", np.array([140, 70, 0]), np.array([180, 255, 255]), (0, 0, 255))
    orange = Color("orange", np.array([0, 100, 0]), np.array([25, 255, 255]), (0, 165, 255))
    yellow = Color("yellow", np.array([20, 70, 0]), np.array([75, 255, 255]), (0, 255, 255))
    green = Color("green", np.array([75, 100, 0]), np.array([95, 255, 255]), (0, 255, 0))
    blue = Color("blue", np.array([95, 150, 0]), np.array([110, 255, 255]), (255, 0, 0))
    purple = Color("purple", np.array([115, 50, 0]), np.array([130, 255, 255]), (255, 0, 255))
    colors = [red, orange, yellow, green, blue, purple]

    def __init__(self, img_path, img_matrix):
        self.img_path = img_path
        self.img_matrix = img_matrix
        self.height, self.width, _ = self.img_matrix.shape
        self.points = None
    
    def crop_image(self):
        height, width, _ = self.img_matrix.shape
        origin_y = int((1/3) * height)
        height = int((1/2) * height)
        origin_x = int((1/3) * width)
        width = int((1/3) * width)
        cropped_image = self.img_matrix[origin_y: origin_y + height, origin_x: origin_x + width]
        return cropped_image
    
    def find_corners(self):
        """
        ASSUMPTION: Runs on a regular image
        This is dependent on there being blue tape in the corners of the image!
        This function is run on the non_homographied image
        But the function should be run on a cropped image ideally
        We also need to have the blue pieces of tape in the top of the image, so not a rotated image
        If we have the blue pieces of tape elsewhere, then we need some way to detect the ar tag...
        """
        corners = []

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(self.img_matrix, cv2.COLOR_BGR2HSV)
        lower_hsv = Image.blue.lower_range
        upper_hsv = Image.blue.upper_range
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Find the contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            # We want to get the blue points
            if radius > 10:
                corners.append(center)

        # Sort the points by y coordinate
        y_sorted = sorted(corners, key = lambda corner: corner[1])
        # The top two corners must be sorted by x and so must the bottom two corners
        corners = sorted(y_sorted[:2], key = lambda corner: corner[0]) + sorted(y_sorted[-2:], key = lambda corner: corner[0])

        # Show the corners
        for corner in corners:
            cv2.circle(self.img_matrix, corner, 5, (0, 255, 0))

        cv2.imshow("Identified Corners", self.img_matrix)
        cv2.waitKey(0)
        cv2.destroyAllWindows()    

        top_left = corners[0]
        top_right = corners[1]
        bottom_right = corners[3]
        bottom_left = corners[2]
        # Return the corners (top_left, top_right, bottom_left, bottom_right)
        return [top_left, top_right, bottom_right, bottom_left]
    
    def find_colored_points(self):
        """
        ASSUMPTION: Runs on a homographied image
        """
        points = []
        for color in Image.colors:
            # Convert the image to HSV color space
            hsv = cv2.cvtColor(self.img_matrix, cv2.COLOR_BGR2HSV)
            lower_hsv = color.lower_range
            upper_hsv = color.upper_range
            plot_color = color.bgr_tuple

            # Create a mask for the color
            mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

            # Find the contours in the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                # We want to filter out the noisy points as well as the corners
                if radius > 7 and not self.in_corners(x, y):
                    points.append([center, color.name])
                    cv2.circle(self.img_matrix, center, radius, plot_color)
        self.points = points
        cv2.imshow("Identified Points", self.img_matrix)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return points
    

    def sort_points(self):
        # Make sure that we have actually identified points
        assert self.points is not None, f"You need to find points first!"

        row_lengths = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]
        assert len(self.points) == sum(row_lengths), f"Not enough points detected! Expected {sum(row_lengths)} but got {len(self.points)}"

        sorted_points = []
        # Sort the points according to their y coordinate
        y_sorted = sorted(self.points, key = lambda point: point[0][1])
        # Note that we sort it based on the idea that the zero coordinate for y is at the top
        
        point_index = 0
        # For each of the rows, sort the points by their x coordinate
        for row_length in row_lengths:
            x_sorted = sorted(y_sorted[point_index: point_index + row_length], key = lambda point: point[0][0])
            sorted_points += x_sorted
            point_index += row_length
        self.points = sorted_points
        return sorted_points
    
    def point_colors(self):
        # Make sure that we have actually identified points
        assert self.points is not None, f"You need to find points first!"
        # Return the colors in a list
        return [point[1] for point in self.points]
    
    def top_down_view(self, width, height):
        corners = self.find_corners()
        top_down_image = rectify_image(self.img_matrix, corners, (500, 500))

        cv2.imshow("Rectified Image", top_down_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return top_down_image

    
    def in_corners(self, x, y):
        """
        Returns whether or not the image coordinate is in one of the corners
        """
        in_bottom_left = (x <= (1/8 * self.width) and y >= (7/8 * self.height))
        in_top_left = (x <= (1/8 * self.width) and y <= (1/8 * self.height))
        in_top_right = (x >= (7/8 * self.width) and y <= (1/8 * self.height))
        in_bottom_right = (x >= (7/8 * self.width) and y >= (7/8 * self.height))
        return in_bottom_left or in_top_left or in_top_right or in_bottom_right


