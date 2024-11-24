import numpy as np

class Border:
    # Class Variables representing directions
    up_slope  = (1, 1)
    down_slope = (-1, -1)
    horizontal = (2, 0)

    def __init__(self, start_point, end_point, direction):
        self.start_point = start_point
        self.end_point = end_point
        self.direction = direction
    def generate_points(self):
        # Generate the difference in x position
        x_steps = np.arange(self.start_point[0], self.end_point[0] + 1, step = self.direction[0])
        # Generate the difference in y position 
        y_steps = np.arange(self.start_point[1], self.end_point[1] + 1, step = self.direction[1])
        return zip(x_steps, y_steps)