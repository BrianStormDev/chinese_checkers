import numpy as np
from Point import Point
from Peg import Peg

class Player:
    def __init__(self, color: str, number: int, directions: list[Point], origin: Point):
        """
        Initialize a player.
        color: string indicating the color of the player (e.g., 'red', 'blue')
        number: integer indicating the number of the player
        directions: an array of strings representing coordinate movements with respect to each player
        initial_pegs: an array of Pegs representing the initial position of pegs a player occupies
        current_pegs: an array of Pegs representing the current position of pegs a player occupies
        """
        self.color = color
        self.number = number
        self.directions = {}
        
        # Intialize the player directions
        self.directions["UL"] = directions[0]
        self.directions["UR"] = directions[1]
        self.directions["R"] = directions[2]
        self.directions["DR"] = directions[3]
        self.directions["DL"] = directions[4]
        self.directions["L"] = directions[5]

        # Set the origin of the pegs and the current state of pegs
        self.initial_pegs = self.initialize_pegs(origin)
        self.current_pegs = self.initial_pegs.copy()
    
    def initialize_pegs(self, origin: Point) -> list[Peg]:
        """
        Initializes the pegs and their position
        origin: Point representing the origin of the positions
        """
        pegs = []
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = origin + self.directions["UL"] * i + self.directions["R"] * j
                cur_peg = Peg(cur_position, True, False, self.color)
                pegs.append(cur_peg)
        return pegs
        
    def reset_pegs(self) -> list[Peg]:
        self.current_pegs = self.initial_pegs.copy()

    def peg_positions(self) -> list[Point]:
        return [peg.position for peg in self.current_pegs]
