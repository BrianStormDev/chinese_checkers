import numpy as np
import matplotlib.pyplot as plt

class Player:
    def __init__(self, color, directions):
        self.color = color
        self.directions = {}
        # Intialize the player directions
        self.directions["UL"] = directions[0]
        self.directions["UR"] = directions[1]
        self.directions["R"] = directions[2]
        self.directions["DR"] = directions[3]
        self.directions["DL"] = directions[4]
        self.directions["L"] = directions[5]

red_directions = [(-1, 1), (1, 1), (2, 0), (1, -1), (-1, -1), (-2, 0)]
orange_directions = [(1, 1), (2, 0), (1, -1), (-1, -1), (-2, 0), (-1, 1)]
blue_directions = [(2, 0), (1, -1), (-1, -1), (-2, 0), (-1, 1), (1, 1)]
yellow_directions = [(1, -1), (-1, -1), (-2, 0), (-1, 1), (1, 1), (2, 0)]
purple_directions = [(-1, -1), (-2, 0), (-1, 1), (1, 1), (2, 0), (1, -1)]
green_directions = [(-2, 0), (-1, 1), (1, 1), (2, 0), (1, -1), (-1, -1)]

