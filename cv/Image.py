from Color import Color
import numpy as np
import cv2

class Image:
    red = Color("red", np.array([160, 70, 0]), np.array([175, 255, 255]), (0, 0, 255))
    orange = Color("orange", np.array([0, 90, 200]), np.array([20, 255, 255]), (0, 165, 255))
    yellow = Color("yellow", np.array([20, 0, 180]), np.array([40, 255, 255]), (0, 255, 255))
    green = Color("green", np.array([75, 100, 0]), np.array([90, 255, 255]), (0, 255, 0))
    blue = Color("blue", np.array([100, 120, 0]), np.array([110, 255, 255]), (255, 0, 0))
    purple = Color("purple", np.array([110, 80, 0]), np.array([130, 150, 255]), (255, 0, 255))
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
        This is dependent on there being blue tape in the corners of the image!
        This function is run on the non_homographied image. But ideally it is a cropped image
        The corner detection works only if the corners are higher than the first blue point on the graph...
        """
        corners = []
        # Booleans that define the corners of the image
        in_bottom_left = (x <= (1/8 * self.width) and y >= (7/8 * self.height))
        in_top_left = (x <= (1/8 * self.width) and y <= (1/8 * self.height))
        in_top_right = (x >= (7/8 * self.width) and y <= (1/8 * self.height))
        in_bottom_right = (x >= (7/8 * self.width) and y >= (7/8 * self.height))

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
            # We want to specifically get points in the corner
            if radius > 7 and self.in_corners(x, y):
                corners.append(center)
                cv2.circle(self.img_matrix, center, radius, (0, 255, 0))

        assert len(corners) == 4, f"Incorrect number of corners, expected 4 but got {len(corners)}"

        cv2.imshow("Identified Corners", self.img_matrix)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Sort the points by y coordinate
        y_sorted = sorted(self.points, key = lambda point: point[0][1])
        # The top two corners must be sorted by x and so must the bottom rwo corners
        corners = sorted(y_sorted[:2], key = lambda point: point[0][0]) + sorted(y_sorted[2:], key = lambda point: point[0][0])
        # Return the corners (top_left, top_right, bottom_left, bottom_right)
        return corners
    
    def find_colored_points(self):
        """
        We assume that this function is run on the homographied image
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
        cv2.imshow("Identified Points", self.img_matrix)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.points = points
        return points
    
    # def sort_points(self):
    #     row_lengths = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]
    #     assert len(self.points) == sum(row_lengths), f"Not enough points detected! Expected {sum(row_lengths)} but got {len(self.points)}"

    #     sorted_points = []
    #     # Sort the points according to their y coordinate
    #     y_sorted = sorted(self.points, key = lambda point: point[1])
    #     # Note that we sort it based on the idea that the zero coordinate for y is at the top
        
    #     point_index = 0
    #     # For each of the rows, sort the points by their x coordinate
    #     for row_length in row_lengths:
    #         x_sorted = sorted(y_sorted[point_index: row_lengths + 1], key = lambda point: point[0])
    #         sorted_points += x_sorted
    #         point_index += row_length
    #     return sorted_points
    
    # def top_down_view(self):
    #     # Here we need to find the corners, which is only going to be used by this function
    #     def corner_finder():
    #         hsv = cv2.cvtColor(self.img_matrix, cv2.COLOR_BGR2HSV)
    #         lower_hsv = Image.blue.lower_range
    #         upper_hsv = Image.blue.upper_range
        
    #     corners = corner_finder()
    
    def in_corners(self, x, y):
        """
        Returns whether or not the image coordinate is in one of the corners
        """
        in_bottom_left = (x <= (1/8 * self.width) and y >= (7/8 * self.height))
        in_top_left = (x <= (1/8 * self.width) and y <= (1/8 * self.height))
        in_top_right = (x >= (7/8 * self.width) and y <= (1/8 * self.height))
        in_bottom_right = (x >= (7/8 * self.width) and y >= (7/8 * self.height))
        return in_bottom_left or in_top_left or in_top_right or in_bottom_right


