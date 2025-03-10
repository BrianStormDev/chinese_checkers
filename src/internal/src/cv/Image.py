from .Color import Color
import numpy as np
import cv2

class Image:
    red = Color("Red", np.array([150, 90, 120]), np.array([179, 255, 255]), (0, 0, 255))
    orange = Color("Darkorange", np.array([0, 100, 0]), np.array([25, 255, 255]), (0, 165, 255))
    yellow = Color("Gold", np.array([20, 70, 0]), np.array([75, 255, 255]), (0, 255, 255))
    green = Color("Green", np.array([75, 100, 0]), np.array([95, 255, 255]), (0, 255, 0))
    blue = Color("Blue", np.array([90, 150, 0]), np.array([110, 255, 255]), (255, 255, 0))
    purple = Color("Purple", np.array([115, 70, 0]), np.array([135, 110, 255]), (255, 0, 255))
    white = Color("Black", np.array([0, 0, 230]), np.array([179, 60, 255]), (0, 0, 0))
    # We also want to include some kind of white threshold
    colors = [red, orange, yellow, green, blue, purple, white]

    def __init__(self, origin_img_path, img_matrix, is_cropped=False, is_rectified=False):
        """
        Initializes information about the Image
        """
        self.origin_img_path = origin_img_path
        self.img_matrix = img_matrix
        self.height, self.width, _ = self.img_matrix.shape
        self.is_cropped = is_cropped
        self.is_rectified = is_rectified
        self.corners = None
        self.points = None
    
    def crop_image(self, origin_x, origin_y, width, height):
        """
        Crops the image according to the specified parameters
        origin_x: x coordinate to start the crop
        origin_y: y coordinate to start the crop
        width: width of the cropped image
        height: height of the cropped image
        """
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
            if cv2.contourArea(contour) > tolerance:
                corners.append(center)

        # Sort the points by y coordinate
        y_sorted = sorted(corners, key = lambda corner: corner[1])
        # The top two corners must be sorted by x and so must the bottom two corners
        corners = sorted(y_sorted[:2], key = lambda corner: corner[0]) + sorted(y_sorted[-2:], key = lambda corner: corner[0])

        # # Show the corners
        # for corner in corners:
        #     cv2.circle(self.img_matrix, corner, 5, (0, 255, 0))

        # cv2.imshow("Identified Corners", self.img_matrix)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()   

        top_left = corners[0]
        top_right = corners[1]
        bottom_right = corners[3]
        bottom_left = corners[2]
        # Return the corners (top_left, top_right, bottom_left, bottom_right)
        self.corners = [top_left, top_right, bottom_right, bottom_left]
        return self.corners
    
    def find_colored_points(self, tolerance):
        """
        ASSUMPTION: Runs on a rectified image
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
                if cv2.contourArea(contour) > tolerance and (not self.in_corners(x, y)):
                    if color is not Image.white:
                        if radius > 4:
                            points.append([center, color.name, radius])
                            cv2.circle(image_copy, center, radius, plot_color)
                    else:
                        points.append([center, color.name, radius])
                        cv2.circle(image_copy, center, radius, plot_color)
        
        # Display the image
        # cv2.imshow('Located Points', image_copy)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        self.points = points
        return points
    
    def sort_points(self):
        """
        Sorts the points from top down left to right
        """
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
        """
        Outputs the colors of each of the points in a list
        """
        # Make sure that we have actually identified points
        assert self.points is not None, f"You need to find points first!"
        # Return the colors in a list
        return [point[1] for point in self.points]
    
    def rectify(self, width, height):
        """
        Takes the corners of the board and uses a homography matrix to get top down image
        rectified_img: Image matrix after being rectified
        """
        assert self.corners != None, "You need to find corners first!"
        src_points = np.array(self.corners)
        dst_points = np.array([(0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1)])
        homography_matrix, _ = cv2.findHomography(src_points, dst_points)
        rectified_img = cv2.warpPerspective(self.img_matrix, homography_matrix, (width, height))
        self.height, self.width, _ = rectified_img.shape
        self.img_matrix = rectified_img
        self.is_rectified = True
        return rectified_img

    def in_corners(self, x, y):
        """
        Returns whether or not the image coordinate is in one of the corners
        """
        in_bottom_left = (x <= (1/5 * self.width) and y >= (4/5 * self.height))
        in_top_left = (x <= (1/5 * self.width) and y <= (1/5 * self.height))
        in_top_right = (x >= (4/5 * self.width) and y <= (1/5 * self.height))
        in_bottom_right = (x >= (4/5 * self.width) and y >= (4/5 * self.height))
        return in_bottom_left or in_top_left or in_top_right or in_bottom_right
    
    def reset_image(self):
        """
        Resets all of the flags for the image
        """
        self.img_matrix = cv2.imread(self.origin_img_path)
        self.is_rectified = False
        self.is_flattened = False
        self.points = None
        self.corners = None