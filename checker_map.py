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
    # TODO Revisit the Origin Points of the triangles, make sure they match with physical board

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

    def move_piece(self, player: Player, starting_peg, move_command: List[str]) -> bool:
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
    game.display_board()
    # game.play_game()
