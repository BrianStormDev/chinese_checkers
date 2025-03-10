#!/usr/bin/env python
from point import Point
from typing import List

class Player:
    def __init__(self, number: int, color: str, origin: Point, directions: List[Point]) -> None:
        """
        Initialize a player.
        number: integer indicating the number of the player
        color: string indicating the color of the player 
        directions: a list of strings representing coordinate movements with respect to each player
        initial_pegs: a list of Pegs representing the initial position of pegs a player occupies
        current_pegs: a list of Pegs representing the current position of pegs a player occupies
        starting_zone_points: a list of Points representing the initial position of each of the player's intial pegs
        """
        self.number = number
        self.color = color
        self.directions = {}
        
        # Intialize the player directions
        self.directions["UL"] = directions[0]
        self.directions["UR"] = directions[1]
        self.directions["R"] = directions[2]
        self.directions["DR"] = directions[3]
        self.directions["DL"] = directions[4]
        self.directions["L"] = directions[5]

        # Set the origin of the pegs and the current state of pegs
        self.starting_zone_points = set()
        self.initial_pieces = self.initialize_pieces(origin)
        self.current_pieces = []
        self.reset_pieces()
    
    def initialize_pieces(self, origin) -> List[Point]:
        """
        Initializes the players pieces
        origin: Point representing the origin of the positions
        """
        pieces = []
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = origin + self.directions["UL"] * i + self.directions["R"] * j
                self.starting_zone_points.add(cur_position)
                pieces.append(cur_position)
        return pieces

    def reset_pieces(self) -> List[Point]:
        """
        Resets the players current_pieces
        Clears their current_pieces and then makes a deep copy of their initial_pieces
        """
        self.current_pieces.clear()
        self.current_pieces = self.initial_pieces.copy()

    def __repr__(self):
        return f'Player {self.number}/{self.color}'

    def __str__(self):
        return f'Player {self.number}/{self.color}'
    
