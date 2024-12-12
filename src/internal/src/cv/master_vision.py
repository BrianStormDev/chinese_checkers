from .Image import Image
import cv2
from ultralytics import YOLO
from .camera_to_board import OK

TOTAL_PEGS = 61 + 60
CONFIDENCE_THRESHOLD = 0.8
class_names = ['Gold', 'Blue', 'Black', 'Purple', 'Green', 'Red', 'Darkorange']

def get_center(box): 
    x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
    center_x, center_y = (x2 - x1) / 2 + x1, (y2 - y1) / 2 + y1
    return center_x, center_y


def detect_boxes(image): 
    model = YOLO("model.pt")
    result = model(image)[0] # TODO: Check if arg should be image path
    boxes = list(result.boxes)
    detected_count = len(boxes)

    assert detected_count >= TOTAL_PEGS, f'{detected_count}/{TOTAL_PEGS} not all pegs detected'

    boxes.sort(key=lambda box: box.conf[0])

    if detected_count > TOTAL_PEGS: 
        for _ in range(detected_count - TOTAL_PEGS): 
            boxes.pop(0)
        print(f'{detected_count - TOTAL_PEGS} extra detections removed')

    low_confidence_boxes = list(filter(lambda box: box.conf[0] <= CONFIDENCE_THRESHOLD, boxes))
    assert not low_confidence_boxes, f'Exists detections with confidence <= {CONFIDENCE_THRESHOLD}'

    return boxes


def add_row(boxes, board): 
    boxes.sort(key=lambda b: get_center(b)[0])
    row = [class_names[int(boxes[i].cls[0])] for i in range(len(boxes))]
    board.append(row)


def create_board(boxes): 
    NUM_ROWS = 17

    # val at ith = # pegs on ith row
    PEGS_PER_ROW = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]

    boxes.sort(key=lambda b: get_center(b)[1])
    board = []

    for i in range(NUM_ROWS): 
        num_boxes_before = sum(PEGS_PER_ROW[:i])
        boxes_that_row = boxes[num_boxes_before : num_boxes_before + PEGS_PER_ROW[i]]
        add_row(boxes_that_row, board)

    return sum(reversed(board))


def image_to_board(raw_image): 
    """
    Input: image
    Output: peg color array
    """

    image = Image(None, raw_image)
    h, w, _ = raw_image.shape
    cropped = image.crop_image(int(0.27 * w), int(0.15 * h), int(0.7 * w), int(0.8 * h)) # TODO: fill in parameters)

    cv2.imshow('Cropped', cropped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    image.find_corners(50) # TODO: adjust in param
    rectified_image = image.rectify(500, 500)

    cv2.imshow('Corrected', rectified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print('Processing...')


    # boxes = detect_boxes(rectified_image)
    # return create_board(boxes)

    m = {
        'blue': "Blue", 
        'yellow': 'Gold', 
        'black': 'Black', 
        'purple': 'Purple', 
        'red': 'Red', 
        'orange': 'Darkorange', 
        'green': 'Green'
    }

    result = OK(rectified_image)
    assert all([i in m for i in result])
    return [m[c] for c in OK(rectified_image)]

if __name__ == '__main__': 
    colors = image_to_board(cv2.imread('/home/cc/ee106a/fa24/class/ee106a-air/ros_workspaces/chinese_checkers/src/ar_tag/src/saved_images/new_image_27.jpg'))
    print(colors)