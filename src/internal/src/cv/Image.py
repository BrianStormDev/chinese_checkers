from Color import Color
import numpy as np
import cv2
from rectify import rectify_image
from point_reader import read_points
from warp_img import compute_homography, warp_image

class Image:
    red = Color("Red", np.array([150, 90, 120]), np.array([179, 255, 255]), (0, 0, 255))
    orange = Color("Darkorange", np.array([0, 100, 0]), np.array([25, 255, 255]), (0, 165, 255))
    yellow = Color("Gold", np.array([20, 70, 0]), np.array([75, 255, 255]), (0, 255, 255))
    green = Color("Green", np.array([75, 100, 0]), np.array([95, 255, 255]), (0, 255, 0))
    blue = Color("Blue", np.array([95, 150, 0]), np.array([110, 255, 255]), (255, 255, 0))
    purple = Color("purple", np.array([115, 50, 0]), np.array([130, 255, 255]), (255, 0, 255))
    white = Color("Black", np.array([0, 0, 230]), np.array([179, 60, 255]), (0, 0, 0))
    # We also want to include some kind of white threshold
    colors = [red, orange, yellow, green, blue, purple, white]

    def __init__(self, origin_img_path, img_matrix, is_cropped=False, is_rectified=False):
        self.origin_img_path = origin_img_path
        self.img_matrix = img_matrix
        self.height, self.width, _ = self.img_matrix.shape
        self.is_cropped = is_cropped
        self.is_rectified = is_rectified
        self.corners = None
        self.points = None
    
    def crop_image(self, origin_x, origin_y, width, height):
        cropped_image = self.img_matrix[origin_y: origin_y + height, origin_x: origin_x + width]
        self.img_matrix = cropped_image
        self.height, self.width, _ = cropped_image.shape
        return cropped_image
    
    def find_corners(self, tolerance):
        """
        ASSUMPTION: Runs on a cropped image
        This is dependent on there being blue tape in the corners of the image!
        This function is run on the non_homographied image
        But the function should be run on a cropped image ideally
        We also need to have the blue pieces of tape in the top of the image, so not a rotated image
        If we have the blue pieces of tape elsewhere, then we need some way to detect the ar tag...
        """
        assert self.is_rectified == False, "This should run on an unprocessed image"
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
            if radius > tolerance:
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
        self.corners = [top_left, top_right, bottom_right, bottom_left]
        return self.corners
    
    def find_colored_points(self, tolerance):
        """
        ASSUMPTION: Runs on a homographied image
        """
        assert self.is_rectified == True, "You need to get the top down image first!"
        points = []
        # All point image
        image_copy = self.img_matrix.copy()
        for color in Image.colors:
            temp_copy = self.img_matrix.copy()
            # Convert the image to HSV color space
            hsv = cv2.cvtColor(temp_copy, cv2.COLOR_BGR2HSV)
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
                if cv2.contourArea(contour) > tolerance and not self.in_corners(x, y):
                    points.append([center, color.name, radius])
                    cv2.circle(image_copy, center, radius, plot_color)
        
        # Display the image
        cv2.imshow('Located Points', image_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        self.points = points
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
    
    def rectify(self, width, height):
        assert self.corners != None, "You need to find corners first!"
        top_down_image = rectify_image(self.img_matrix, self.corners, (width, height))
        self.height, self.width, _ = top_down_image.shape
        self.img_matrix = top_down_image
        self.is_rectified = True
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
    
    def reset_image(self):
        self.img_matrix = cv2.imread(self.origin_img_path)
        self.is_rectified = False
        self.is_flattened = False
        self.points = None
        self.corners = None