from ultralytics import YOLO
import cv2
import os, sys
from pathlib import Path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TOTAL_PEGS = 61 + 60
CONFIDENCE_THRESHOLD = 0.7
EMPTY = 'empty'
class_names = ['yellow', 'blue', EMPTY, 'purple', 'green', 'red', 'orange']

cur_dir = Path(__file__).parent

model = YOLO(str(cur_dir / "model_2.pt"), verbose=False)

# image_path = "new_rectified_images/new_data/image_12.jpg" 
image_path = "/home/cc/ee106a/fa24/class/ee106a-air/ros_workspaces/chinese_checkers/src/internal/src/cv/special.jpg"
image = cv2.imread(image_path)

def OK(image):
    # print('Loading done')

    # Perform object detection
    result = model(image, verbose=False)[0]

    # print('Object detection done')

    boxes = list(result.boxes)
    detected_count = len(boxes)

    assert detected_count >= TOTAL_PEGS, f'{detected_count}/{TOTAL_PEGS} not all pegs detected'

    boxes.sort(key=lambda box: box.conf[0])

    if detected_count > TOTAL_PEGS: 
        for _ in range(detected_count - TOTAL_PEGS): 
            boxes.pop(0)
        # print(f'{detected_count - TOTAL_PEGS} extra detections removed')

    low_confidence_boxes = list(filter(lambda box: box.conf[0] <= CONFIDENCE_THRESHOLD, boxes))

    assert not low_confidence_boxes, f'Exists detections with confidence <= {CONFIDENCE_THRESHOLD}'



    NUM_ROWS = 17
    MAX_PEGS_PER_ROW = 25
    CENTER_COL = 13

    # val at ith = # pegs on ith row
    PEGS_PER_ROW = [1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1]

    def get_center(box): 
        x1, y1, x2, y2 = [int(i) for i in box.xyxy[0]]
        center_x, center_y = (x2 - x1) / 2 + x1, (y2 - y1) / 2 + y1
        return center_x, center_y

    boxes.sort(key=lambda b: get_center(b)[1])
    board = []

    def add_row(boxes, row_num, board): 
        row = ['white' for i in range(MAX_PEGS_PER_ROW)]
        boxes.sort(key=lambda b: get_center(b)[0])

        start_ind = CENTER_COL - len(boxes)

        for i in range(len(boxes)): 
            x = start_ind + (i * 2)
            is_empty = class_names[int(boxes[i].cls[0])] == EMPTY
            color = 'black' if is_empty else class_names[int(boxes[i].cls[0])]
            row[x] = color

        board.append(row)

    def invert_board(board): 
        board = reversed(board)

    for i in range(NUM_ROWS): 
        num_boxes_before = sum(PEGS_PER_ROW[:i])
        boxes_that_row = boxes[num_boxes_before : num_boxes_before + PEGS_PER_ROW[i]]
        add_row(boxes_that_row, i, board)

    invert_board(board)

    # def display_board(self):
    #     colors = []
    #     x_coords = []
    #     y_coords = []
    #     for row in board: 
    #         for peg in row: 
    #             colors.append(peg.color)
    #             x_coords.append(peg.position.x)
    #             y_coords.append(peg.position.y)

    #     # Plot the points with their colors
    #     plt.scatter(x_coords, y_coords, c=colors)
    #     plt.xlabel('X-axis')
    #     plt.xticks(list(range(25)))
    #     plt.ylabel('Y-axis')
    #     plt.yticks(list(range(17)))
    #     plt.title('Checker Board Visualization')
    #     plt.grid()
    #     plt.show()

    # display_board(board)

    return [color for row in board for color in row if color != 'white']

if __name__ == '__main__': 
    OK(image)

