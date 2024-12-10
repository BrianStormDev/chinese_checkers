import cv2

def crop_image(image, origin_x, origin_y, width, height):
    """
    image: An image object (really just a matrix of numbers)
    origin_x: The x position to start the image crop
    origin_y: The y position to start the image crop
    width: The width of the image crop
    height: The height of the image crop
    """
    return image[origin_y: origin_y + height, origin_x: origin_x + width]

