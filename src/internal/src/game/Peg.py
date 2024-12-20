#!/usr/bin/env python
from .Point import Point

class Peg:
    """
    Peg class representing the holes on the Chinese Checkers board.
    """
    def __init__(self, position: Point, color: str, in_board: bool, is_empty: bool) -> None:
        """
        position: Point indicating the position of the peg
        color: string indicating the color of the peg
        in_board: boolean indicating if the peg is in the playable region
        is_empty: boolean indicating if the peg has a ball in it
        """
        self.position = position
        self.color = color
        self.in_board = in_board
        self.is_empty = is_empty
    
    def copy(self):
        """
        Creates a copy of the peg
        """
        return Peg(self.position, self.color, self.in_board, self.is_empty)
    
    def __repr__(self):
        position = f'Position: {self.position} | '
        color = f'Color: {self.color} | '
        in_board = f'In Board: {self.in_board} | '
        is_empty = f'Is Empty: {self.is_empty}'
        return position + color + in_board + is_empty
    
    def __str__(self):
        position = f'Position: {self.position} | '
        color = f'Color: {self.color} | '
        in_board = f'In Board: {self.in_board} | '
        is_empty = f'Is Empty: {self.is_empty}'
        return position + color + in_board + is_empty