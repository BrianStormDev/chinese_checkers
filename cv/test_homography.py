import cv2
import numpy as np

# List to store points
points = []

# A callback function that fill get the points any time a user clicks on a point in the image
def get_points(event, x, y, flags, param):
    """
    Mouse callback function to record points on mouse click.
    """
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        print(f"Point selected: {x}, {y}")
        if len(points) <= 4:
            cv2.circle(temp_image, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Select Points", temp_image)
            

def main():
    global points, temp_image

    # Load the input image
    image = cv2.imread('image_1.jpg')
    temp_image = image.copy()

    # Display the image and set up mouse callback
    cv2.imshow("Select Points", image)
    cv2.setMouseCallback("Select Points", get_points)
    print("Select four points on the image in clockwise order, starting from the top-left corner.")
    
    # Wait until four points are selected
    while len(points) < 4:
        cv2.waitKey(1)
    
    cv2.destroyAllWindows()

    # Define destination points (desired rectangle for rectification)
    width, height = 400, 300  # Define desired output dimensions
    dst_points = np.array([
        [0, 0],         # Top-left
        [width-1, 0],   # Top-right
        [width-1, height-1],  # Bottom-right
        [0, height-1]   # Bottom-left
    ], dtype=np.float32)

    # Convert points to NumPy array
    src_points = np.array(points, dtype=np.float32)

    # Compute the homography matrix
    H, _ = cv2.findHomography(src_points, dst_points)

    # Warp the image
    rectified_image = cv2.warpPerspective(image, H, (width, height))

    # Display and save the rectified image
    cv2.imshow("Rectified Image", rectified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('rectified_document.jpg', rectified_image)
    print("Rectified image saved as 'rectified_document.jpg'.")

if __name__ == "__main__":
    main()
