from Point import Point

class Peg:
    def __init__(self, position: Point, in_board: bool, is_empty: bool, color: str):
        """
        Initialize a peg.
        is_empty: boolean indicating if the peg is empty
        in_board: boolean indicating if the peg is a slot on the board
        color: string indicating the color of the piece on the peg (e.g., 'red', 'blue')
        position: Point indicating the position of the peg
        """
        self.position = position
        self.in_board = in_board
        self.is_empty = is_empty
        self.color = color
        
    def __repr__(self):
        # position_str = f'Position: {str(self.position)}'
        # color_str = f'Color: {self.color} | '
        # in_board = f'In Board: {self.in_board} | '
        # is_empty = f'Is Empty: {self.is_empty} '
        return str(self.position) + self.color # + color_str + in_board + is_empty    
    
    def __str__(self):
        position_str = f'Position: {str(self.position)} | '
        color_str = f'Color: {self.color} | '
        in_board = f'In Board: {self.in_board} | '
        is_empty = f'Is Empty: {self.is_empty} '
        return position_str + color_str + in_board + is_empty