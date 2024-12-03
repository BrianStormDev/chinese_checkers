import numpy as np
import matplotlib.pyplot as plt
from Peg import Peg
from typing import List
from Player import Player
from Point import Point

X_DIM = 25
Y_DIM = 17

# The key thing to note is that a chinese checkers board has players, not a player having a board
# But each player has a set of pieces, representing what pegs they occupy
# The board associates a triangle as the start for each of the players
# The board associates a triangle as the end for each of the players

class ChineseCheckersBoard:
    # Information about players and directions to store for later
    # Class attributes
    yellow_directions = [Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0)]
    red_directions = [Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0)]
    orange_directions = [Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1)]
    purple_directions = [Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1)]
    blue_directions = [Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1)]
    green_directions = [Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1)]

    player_1 = Player("Yellow", 1, yellow_directions, Point(12, 16), Point(12, 0))
    player_2 = Player("Red", 2, red_directions, Point(12, 0), Point(12, 16))
    player_3 = Player("Orange", 3, orange_directions, Point(0, 4), Point(24, 12))
    player_4 = Player("Purple", 4, purple_directions, Point(24, 12), Point(0, 4))
    player_5 = Player("Blue", 5, blue_directions, Point(0, 12), Point(24, 4))
    player_6 = Player("Green", 6, green_directions, Point(24, 4), Point(0, 12))

    def __init__(self, x_dim: int, y_dim: int, num_players=2):
        """
        Initialize a Board
        x_dim: An integer representing the x dimension of the board
        y_dim: An integer representing the y dimension of the board
        """
        self.num_players = num_players
        self.all_players = [self.player_1, self.player_2, self.player_3, self.player_4, self.player_5, self.player_6]
        if num_players == 2: 
            self.players = self.all_players[2]
        elif num_players == 4: 
            self.players = self.all_players[4]
        elif num_players == 6: 
            self.players = self.all_players[6]
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.board = self.initialize_board()

    def initialize_board(self):
        """
        Initialize a Board
        """
        # The idea is that any untouchable spaces are white, so we can just make the whole board white, then fill in the gaps
        board = np.ndarray((self.x_dim, self.y_dim), dtype=object)
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                board[i, j] = Peg(Point(i, j), False, True, "white")

        # # Initialize the hexagon for the board
        # bottom_hexagon_origin = Point(8, 4)
        # for i in range(5):
        #     board[bottom_hexagon_origin.x, bottom_hexagon_origin.y] = Peg(bottom_hexagon_origin, False, True, "black")
        #     for j in range(5 + i):
        #         cur_position = bottom_hexagon_origin + (i * Point(-1, 1)) + (j * Point(2, 0))
        #         board[cur_position.x, cur_position.y] = Peg(cur_position, False, True, "Black")
        # top_hexagon_origin = Point(8, 12)
        # for i in range(4):
        #     board[top_hexagon_origin.x, top_hexagon_origin.y] = Peg(top_hexagon_origin, False, True, "black")
        #     for j in range(5 + i):
        #         cur_position = top_hexagon_origin + (i * Point(-1, -1)) + (j * Point(2, 0))
        #         board[cur_position.x, cur_position.y] = Peg(cur_position, False, True, "Black")

        # Initialize the hexagon for the board
        hexagon_origin = Point(12, 8)
        for radii in [0, 2, 4, 6, 8]:
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        board[x + hexagon_origin.x, y + hexagon_origin.y] =  Peg(Point(x, y) + hexagon_origin, False, True, "Black")

        # Initialize the players for the board
        for player in self.all_players:
            for peg in player.current_pegs:
                board[peg.position.x, peg.position.y] = peg

        return board

    def display_board(self):
        """Display the board using matplotlib"""
        colors = []
        x_coords = []
        y_coords = []
        flatArray = self.board.flatten()
        for peg in flatArray:
            colors.append(peg.color)
            x_coords.append(peg.position.x)
            y_coords.append(peg.position.y)

        # Plot the points with their colors
        plt.scatter(x_coords, y_coords, c=colors)
        plt.xlabel('X-axis')
        plt.xticks(list(range(self.x_dim)))
        plt.ylabel('Y-axis')
        plt.yticks(list(range(self.y_dim)))
        plt.title('Checker Board Visualization')
        plt.grid()
        plt.show()

    def valid_moves(self, player: Player):
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        # We need to figure out all the valid maneuvers (list of start and end Points)

        # You can either do a chain of jumps, or a singular move
        # If you jump back to the original point, end recursion

        # player.current_pegs refers to the pegs of the current player
        # peg refers to the piece we are looking at 
        # peg.peg_position refers to the current_position of the peg

        # You might jump back to the same spot!!!
        # Figure out what the recursive case
        # Two cases of movements:
        

        def valid_jumps_from_point(origin_pos: Point, current_pos: Point) -> list:
            """
            origin_pos: indicates the initial position of the peg
            current_pos: indicates the current peg we are looking at
            direction: Point indicating which direction we just came from 
            jumping: indicates whether or not we are jumping 
            """
            # if we end back into the state we were at, then we can end recursion there
            jumps = set()

            for move_code in player.directions: # For each possible direction
                direction = player.directions[move_code] # Get the direction
                one_move_pos = current_pos + direction # Get the one_move_position 
                jump_move_pos = current_pos + (2 * direction) # Get the jump_move_position
                
                # Determine if we are allowed to make a jump:
                # 1. Target position is in bounds
                # 2. Target position is empty
                # 3. The adjacent location (one_move_pos) has a piece next to it
                if self.in_bounds(jump_move_pos) and self.is_empty(jump_move_pos) and (not self.is_empty(one_move_pos)):
                    move_tuple = (origin_pos, jump_move_pos)
                    # Determine if we stop jumping:
                    # 1. If the move_tuple is in moves, we don't make the recursive jump
                    # 2. If the current_pos is equal to the origin_pos, we don't make the recursive jump
                    if move_tuple not in jumps and (current_pos != origin_pos): 
                        jumps.add(move_tuple) # We can add a valid move
                        jumps.update(valid_jumps_from_point(origin_pos, jump_move_pos)) # Add all the other valid moves
            return jumps
        
        all_moves = set()

        for peg in player.current_pegs: # For each of the pegs of the current player
            # First determine if we can make any moves one step away 
            for move_code in player.directions: # For each possible direction
                direction = player.directions[move_code] # Get the direction
                origin_pos = peg.peg_position() # Get the current position of the peg

                # Determine if the singular movement is possible
                # 1. Target position is in bounds
                # 2. Target position is empty
                single_move_pos = origin_pos + direction
                if self.in_bounds(single_move_pos) and self.is_empty(single_move_pos):
                    move_tuple = (origin_pos, single_move_pos)
                    all_moves.add(move_tuple)

                # Now figure out if we can make any jump movements
                # 1. Target position is in bounds
                # 2. Target position is empty
                # 3. The adjacent location (one_move_pos) has a piece next to it
                jump_move_pos = origin_pos + (2 * direction)
                if self.in_bounds(jump_move_pos) and self.is_empty(jump_move_pos) and (not self.is_empty(single_move_pos)):
                    move_tuple = (origin_pos, jump_move_pos)
                    all_moves.add(move_tuple)
                    all_moves.update(valid_jumps_from_point(origin_pos, jump_move_pos))

        return list(all_moves)
        
    def move_piece(self, player: Player, starting_peg, move_command: list[str]) -> bool:
        """Attempt to move a piece for a player"""
        # Anytime we move a piece, the starting_peg must be assigned the color black
        if self.is_valid_move_command(player, starting_peg, move_command):
            pass
        
    def is_valid_move(self, player: Player, starting_peg, move: str) -> bool:
        """Check if the move is valid."""
        """"move is a string, that maps to a player direction"""
        direction = player.directions[move]
        target_x = starting_peg.position[0] + direction[0]
        target_y = starting_peg.position[1] + direction[1]
        target_peg = self.board[self.center_coord_to_bot_coord(target_x, target_y)]
        return target_peg.is_empty == True and target_peg.in_board
        
    def check_winner(self, player: Player) -> bool:
        """Check if a player has won."""
        # For every point in the player's "endzone", we check if that point has the players color, if true, return true
        endpoints = player.endpoints
        for point in endpoints:
            if (self.board[point.x, point.y].color != player.color):
                return False
        return True
    
    def peg_at_position(self, position: Point) -> Peg:
        """Return the Peg located at the position"""
        return self.board[position.x][position.y]
    
    def is_empty(self, position: Point) -> bool:
        """Return whether or not there is Peg located at a certain position"""
        if self.board[position.x][position.y].color == "Black":
            return True
        else: # If the Peg is any other color it is either out of bounds or occupied
            return False
        
    def in_bounds(self, position: Point) -> bool:
        """Return whether or not the current position is in bounds"""
        if position.x >= self.x_dim or position.x <= 0 or position.y >= self.y_dim or position.y <= 0:
            return False
        else:
            return True
        
    def play_game(self):
        """Main game loop."""
        self.display_board()
        current_player = 1
        while True:
            print(f"Player {current_player}'s turn.")
            pass

if __name__ == "__main__":
    game = ChineseCheckersBoard(X_DIM, Y_DIM, 2)
    print(game.check_winner(game.player_1))
    print(len(game.valid_moves(game.player_1)))
    # game.display_board()
    # game.play_game()
