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

    def place_piece(self, color):
        """
        Place a piece on this peg.
        color: string indicating the color of the piece on the peg (e.g., 'red', 'blue')
        """
        if not self.is_empty:
            raise ValueError("Peg is already occupied.")
        self.is_empty = False
        self.color = color

    def remove_piece(self):
        """Remove a piece from this peg."""
        if self.is_empty:
            raise ValueError("Peg is already empty.")
        self.is_empty = True
        self.color = None

    def peg_position(self):
        return Point(self.position.x, self.position.y)
        
    def __str__(self):
        position_str = f'Position: {str(self.position)}'
        color_str = f'Color: {self.color}'
        return position_str + " " + color_str