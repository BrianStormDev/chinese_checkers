#!/usr/bin/env python
from Point import Point

class Peg:
    def __init__(self, position: Point, color: str, in_board: bool, is_empty: bool) -> None:
        """
        Initialize a peg.
        position: Point indicating the position of the peg
        color: string indicating the color of the peg
        in_board: boolean indicating if the peg is a slot on the board
        is_empty: boolean indicating if the peg is empty
        """
        self.position = position
        self.color = color
        self.in_board = in_board
        self.is_empty = is_empty
        
    def copy(self):
        return Peg(self.position, self.color, self.in_board, self.is_empty)
    
    def __repr__(self):
        position_str = f'Position: {str(self.position)} | '
        color_str = f'Color: {self.color} | '
        in_board = f'In Board: {self.in_board} | '
        is_empty = f'Is Empty: {self.is_empty} '
        return position_str + color_str + in_board + is_empty
    
    def __str__(self):
        position_str = f'Position: {str(self.position)} | '
        color_str = f'Color: {self.color} | '
        in_board = f'In Board: {self.in_board} | '
        is_empty = f'Is Empty: {self.is_empty} '
        return position_str + color_str + in_board + is_empty
    
    def change_y(self, new_y): 
        self.position.y = new_y