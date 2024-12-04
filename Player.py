import numpy as np
from Point import Point
from typing import List
from Peg import Peg

class Player:
    def __init__(self, color: str, number: int, directions: List[Point], origin: Point, opposite_origin: Point=None):
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
        self.opposite_directions = {}
        self.origin = origin
        self.opposite_origin = opposite_origin
        
        # Intialize the player directions
        self.directions["UL"] = directions[0]
        self.directions["UR"] = directions[1]
        self.directions["R"] = directions[2]
        self.directions["DR"] = directions[3]
        self.directions["DL"] = directions[4]
        self.directions["L"] = directions[5]

        # Initialize the opposite side's directions
        self.opposite_directions["UL"] = directions[3]
        self.opposite_directions["UR"] = directions[4]
        self.opposite_directions["R"] = directions[5]
        self.opposite_directions["DR"] = directions[0]
        self.opposite_directions["DL"] = directions[1]
        self.opposite_directions["L"] = directions[2]
        
        # Set the origin of the pegs and the current state of pegs
        self.initial_pegs = self.initialize_pegs()
        self.current_pegs = self.initial_pegs.copy()
        self.endpoints = self.initialize_opposite_pegs()
    
    def initialize_pegs(self) -> List[Peg]:
        """
        Initializes the pegs and their position
        origin: Point representing the origin of the positions
        """
        pegs = []
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = self.origin + self.directions["UL"] * i + self.directions["R"] * j
                cur_peg = Peg(cur_position, True, False, self.color)
                pegs.append(cur_peg)
        return pegs
        
    def initialize_opposite_pegs(self):
        endPoints = []
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = self.opposite_origin + self.opposite_directions["UL"] * i + self.opposite_directions["R"] * j
                endPoints.append(cur_position)
        return endPoints

    def reset_pegs(self) -> List[Peg]:
        self.current_pegs = self.initial_pegs.copy()

    def peg_positions(self) -> List[Point]:
        return [peg.position for peg in self.current_pegs]
    
    def peg_at_position(self, position: Point) -> Peg:
        return self.current_pegs(self.peg_index_at_position(position))
    
    def peg_index_at_position(self, position: Point) -> Peg:
        for i in len(self.current_pegs):
            if self.current_pegs[i].position == position:
                return i
        print("There is either no peg or this peg does not belong to player")
        return -1 

    def move_peg(self, position: Point, target_position: Point):
        """
        Assuming we pass in a valid move, move the peg accordingly
        """
        peg_index = self.peg_index_at_position(position)
        if peg_index != -1:
            self.current_pegs[peg_index].position = target_position
        print("There is either no peg or this peg does not belong to player")

    def __repr__(self):
            return f'{self.color}'