import cv2
import numpy as np

def detect_corners(image):
    """
    Detect corners in the image using the Shi-Tomasi corner detection method.
    
    Args:
        image: The input image.
    
    Returns:
        corners: The list of corner points detected in the image.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect corners using Shi-Tomasi (Good Features to Track)
    corners = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.01, minDistance=10)

    print(corners)
    
    # Convert the points to integer values for easier processing
    corners = np.int8(corners)

    # Draw the corners on the image
    for corner in corners:
        x, y = corner.ravel()
        cv2.circle(image, (x, y), 3, (0, 255, 0), -1)

    return corners, image



# def select_points(image):
#     """
#     Allow the user to select points interactively on an image.
    
#     Args:
#         image: The input image.

#     Returns:
#         points: A list of four selected points in clockwise order.
#     """
#     points = []
#     temp_image = image.copy()

#     def get_points(event, x, y, flags, param):
#         """
#         Mouse callback function to record points on mouse click.
#         """
#         if event == cv2.EVENT_LBUTTONDOWN:
#             points.append((x, y))
#             print(f"Point selected: {x}, {y}")
#             cv2.circle(temp_image, (x, y), 5, (0, 255, 0), -1)
#             cv2.imshow("Select Points", temp_image)

#     # Display the image and set up the mouse callback
#     cv2.imshow("Select Points", image)
#     cv2.setMouseCallback("Select Points", get_points)
#     print("Select four points on the image in clockwise order, starting from the top-left corner.")

#     # Wait for the user to select four points
#     while len(points) < 4:
#         cv2.waitKey(1)

#     cv2.destroyAllWindows()
#     return points

# def compute_homography(image, src_points, width, height):
#     """
#     Compute and apply a homography transformation to warp the image.

#     Args:
#         image: The input image.
#         src_points: A list of four source points.
#         width: The width of the rectified output image.
#         height: The height of the rectified output image.

#     Returns:
#         rectified_image: The rectified image.
#     """
#     # Define destination points for the desired rectangle
#     dst_points = np.array([
#         [0, 0], # Top-left
#         [width - 1, 0], # Top-right
#         [width - 1, height - 1], # Bottom-right
#         [0, height - 1] # Bottom-left
#     ], dtype=np.float32)

#     # Convert source points to NumPy array
#     src_points = np.array(src_points, dtype=np.float32)

#     # Compute the homography matrix
#     H, _ = cv2.findHomography(src_points, dst_points)

#     # Warp the image using the homography matrix
#     rectified_image = cv2.warpPerspective(image, H, (width, height))
#     return rectified_image

def main():
    # Load the input image
    image = cv2.imread("cv\cropped_image.jpg")

    # Step 1: Detect corners
    corners, image_with_corners = detect_corners(image.copy())
    cv2.imshow("Corners Detected", image_with_corners)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#     # Step 2: Select points interactively (for simplicity, we can allow the user to pick from the detected corners)
#     print("Now, select four points in clockwise order from the detected corners.")
#     src_points = select_points(image_with_corners)

#     # Step 3: Define desired output dimensions
#     output_width, output_height = 400, 300

#     # Step 4: Compute homography and warp the image
#     rectified_image = compute_homography(image, src_points, output_width, output_height)

#     # Display and save the rectified image
#     cv2.imshow("Rectified Image", rectified_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
#     cv2.imwrite("rectified_document.jpg", rectified_image)
#     print("Rectified image saved as 'rectified_document.jpg'.")

if __name__ == "__main__":
    main()
