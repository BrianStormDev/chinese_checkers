# import cv2
# import numpy as np

# def find_board_corners(image_path):
#     # Read the image
#     image = cv2.imread(image_path)
#     if image is None:
#         raise FileNotFoundError(f"Unable to load image from {image_path}")
    
#     # Convert to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     cv2.imshow("Detected Board Corners", gray)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     # Optional: Apply a Gaussian blur to reduce noise
#     gray = cv2.GaussianBlur(gray, (5, 5), 0)

#     cv2.imshow("Detected Board Corners", gray)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     # Use Canny edge detection or thresholding to find edges
#     # edges = cv2.Canny(gray, 50, 150)
#     edges = cv2.Canny(gray, 30, 100)  # Try lowering first threshold or adjusting both


#     cv2.imshow("Detected Board Corners", edges)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     # Find contours in the edge map
#     contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
#     # If no contours found, raise an error
#     if not contours:
#         raise ValueError("No contours found. Try adjusting parameters.")
    
#     # Sort contours by area (largest first)
#     contours = sorted(contours, key=cv2.contourArea, reverse=True)

#     # print(contours)
    
#     board_corners = None
    
#     for cnt in contours:
#         # Approximate the contour to a polygon
#         epsilon = 0.02 * cv2.arcLength(cnt, True)
#         approx = cv2.approxPolyDP(cnt, epsilon, True)
        
#         # If the approximated contour has 4 points, we probably found the board
#         if len(approx) == 4:
#             board_corners = approx
#             break
    
#     if board_corners is None:
#         raise ValueError("No rectangular board could be identified. Adjust parameters or ensure the board is clearly visible.")
    
#     # board_corners is a set of four points in the format: [[[x1, y1]], [[x2, y2]], [[x3, y3]], [[x4, y4]]]
#     # Convert it to a more convenient list of tuples
#     corners = [(pt[0][0], pt[0][1]) for pt in board_corners]
    
#     # Draw the corners on the image for visualization (optional)
#     for corner in corners:
#         cv2.circle(image, corner, 10, (0, 0, 255), -1)
    
#     # Show the result (for debugging)
#     cv2.imshow("Detected Board Corners", image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     return corners

# if __name__ == "__main__":
#     # Example usage:
#     # Make sure you have a correct path to your input image
#     image_path = "new_image_6.jpg"
#     corners = find_board_corners(image_path)
#     print("Detected corners:", corners)





import cv2
import numpy as np
import math

def line_angle(line):
    # Compute the angle of a line segment (x1, y1, x2, y2)
    x1, y1, x2, y2 = line[0]
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def find_intersection(L1, L2):
    # Each line is [x1, y1, x2, y2]
    x1, y1, x2, y2 = L1[0]
    x3, y3, x4, y4 = L2[0]

    denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
    if denom == 0:
        return None  # Lines are parallel or coincident

    # Intersection formula
    intersect_x = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / denom
    intersect_y = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / denom
    return (int(intersect_x), int(intersect_y))

def find_board_corners(image_path):
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Preprocessing might be needed: blur, threshold, etc.
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Detect lines using HoughLinesP
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=80, minLineLength=100, maxLineGap=20)

    if lines is None or len(lines) < 4:
        raise ValueError("Not enough lines detected to form a rectangle.")

    # Separate lines roughly into vertical and horizontal based on their angle
    horizontals = []
    verticals = []
    for line in lines:
        angle = line_angle(line)
        # Near horizontal: angle ~ 0 or 180
        # Near vertical: angle ~ 90 or -90
        if -10 < angle < 10 or 170 < angle < 190:  
            horizontals.append(line)
        elif 80 < angle < 100 or -100 < angle < -80:
            verticals.append(line)

    # Now we ideally have a set of horizontal and vertical lines. 
    # If too many, pick the longest or the ones that form a rectangle around a region of interest.
    if len(horizontals) < 2 or len(verticals) < 2:
        raise ValueError("Not enough horizontal/vertical lines found to identify a board.")

    # Sort lines by length (longest first) to guess that board edges are among the longest
    def line_length(line):
        x1, y1, x2, y2 = line[0]
        return (x2 - x1)**2 + (y2 - y1)**2

    horizontals.sort(key=line_length, reverse=True)
    verticals.sort(key=line_length, reverse=True)

    # Take top 2 longest horizontals and top 2 longest verticals
    horizontals = horizontals[:2]
    verticals = verticals[:2]

    # Compute the four intersections
    # The board corners should be intersection of each horizontal line with each vertical line
    corners = []
    for h in horizontals:
        for v in verticals:
            pt = find_intersection(h, v)
            if pt:
                corners.append(pt)

    # corners might not be in order, sort them (e.g., by x+y) to get consistent order
    # Sort by y then x to have a consistent order (top-left, top-right, bottom-left, bottom-right)
    corners = sorted(corners, key=lambda p: (p[1], p[0]))

    # Visualize for debugging
    for c in corners:
        cv2.circle(image, c, 10, (0,0,255), -1)

    # cv2.imshow("Corners", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return corners

if __name__ == "__main__":
    path = "your_edge_image.jpg"
    try:
        board_corners = find_board_corners(path)
        print("Detected board corners:", board_corners)
    except ValueError as e:
        print("Error:", e)
