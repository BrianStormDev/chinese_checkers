import numpy as np

class Item:
    def __init__(self, color_name, lower_bound, upper_bound):
        self.color = color_name
        self.lower_hsv = lower_bound
        self.upper_hsv = upper_bound

red = Item("Red", np.array())
orange = Item("Orange", np.array())
yellow = Item("Yellow", np.array())
green = Item("Green", np.array())
blue = Item("Blue", np.array())
purple = Item("Purple", np.array())
    
