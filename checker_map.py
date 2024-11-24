import numpy as np
import matplotlib.pyplot as plt
from Peg import Peg
from Player import Player
from Point import Point

X_DIM = 27
Y_DIM = 18

# The key thing to note is that a chinese checkers board has players, not a player having a board
# But each player has a set of pieces, representing what pegs they occupy
# The board associates a triangle as the start for each of the players
# The board associates a triangle as the end for each of the players

class ChineseCheckersBoard:
    # TODO Revisit the Origin Points of the triangles, make sure they match with physical board

    # Information about players and directions to store for later
    red_directions = [Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0)]
    orange_directions = [Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1)]
    blue_directions = [Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1)]
    yellow_directions = [Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0)]
    purple_directions = [Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1)]
    green_directions = [Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1)]

    player_1 = Player("Red", 1, red_directions, Point(13, 0))
    player_2 = Player("Orange", 2, orange_directions, Point(1, 4))
    player_3 = Player("Blue", 3, blue_directions, Point(1, 12))
    player_4 = Player("Yellow", 4, yellow_directions, Point(13, 16))
    player_5 = Player("Purple", 5, purple_directions, Point(25, 12))
    player_6 = Player("Green", 6, green_directions, Point(25, 4))

    def __init__(self, x_dim, y_dim, num_players=2):
        """
        Initialize a Board
        x_dim: An integer representing the x dimension of the board
        y_dim: An integer representing the y dimension of the board
        """
        self.num_players = num_players
        self.players = [self.player_1, self.player_2, self.player_3, self.player_4, self.player_5, self.player_6]
        self.x_dim = x_dim
        self.y_dim = y_dim
        self.board = self.initialize_board()

    def initialize_board(self):
        """
        Initialize a Board
        """
        # The idea is that any untouchable spaces are white, so we can just make the whole board
        # white, then fill in the gaps
        board = np.ndarray((self.x_dim, self.y_dim), dtype=object)

        # Initialize the hexagon for the board
        bottom_hexagon_origin = Point(9, 4)
        for i in range(5):
            board[bottom_hexagon_origin.x, bottom_hexagon_origin.y] = Peg(bottom_hexagon_origin, False, True, "black")
            for j in range(5 + i):
                cur_position = bottom_hexagon_origin + (i * Point(-1, 1)) + (j * Point(2, 0))
                board[cur_position.x, cur_position.y] = Peg(cur_position, False, True, "Black")
        top_hexagon_origin = Point(9, 12)
        for i in range(4):
            board[top_hexagon_origin.x, top_hexagon_origin.y] = Peg(top_hexagon_origin, False, True, "black")
            for j in range(5 + i):
                cur_position = top_hexagon_origin + (i * Point(-1, -1)) + (j * Point(2, 0))
                board[cur_position.x, cur_position.y] = Peg(cur_position, False, True, "Black")

        # Initialize the players for the board
        for player in self.players:
            for peg in player.current_pegs:
                board[peg.position.x, peg.position.y] = peg
        
        return board
        
    #     for x in range(self.x_dim):
    #         for y in range(self.y_dim):
    #             x_prime, y_prime = self.bot_coord_to_center_coord(x, y)
    #             board[x, y] = Peg(x_prime, y_prime, False, True, "black")
        
    #     for j in range(-4, 5):
    #         for i in np.arange(- 8 + abs(j), 9 - abs(j), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "pink")

    #     for j in range(1, 5):
    #         for i in np.arange(-9 - abs(j) + 1, -9 + abs(j), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "blue")

    #     for j in range(1, 5):
    #         for i in np.arange(9 - abs(j) + 1, 9 + abs(j), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "purple")

    #     for j in range(-4, 0):
    #         for i in np.arange(-9 - abs(j) + 1, -9 + abs(j), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "orange")

    #     for j in range(-4, 0):
    #         for i in np.arange(9 - abs(j) + 1, 9 + abs(j), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "green")
        
    #     for j in range(5, 9):
    #         for i in np.arange(- abs(j - 9) + 1, abs(j - 9), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "yellow")

    #     for j in range(-8, -4):
    #         for i in np.arange(- abs(j + 9) + 1, abs(j + 9), 2):
    #             x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
    #             board[x_prime, y_prime] = Peg(i, j, True, True, "red")
    #     return board
        
    # def center_coord_to_corner_coord(self, coordinate):
    #     """
    #     coordinate: tuple indicating a position w.r.t origin at center of board
    #     returns: tuple indicating a position w.r.t origin at corner of board
    #     """
    #     return coordinate[0] + self.x_dim // 2, coordinate[1] + self.y_dim // 2

    # def corner_coord_to_center_coord(self, coordinate):
    #     """
    #     coordinate: tuple indicating a position w.r.t origin at corner of board
    #     returns: tuple indicating a position w.r.t origin at center of board
    #     """
    #     return coordinate[0] - self.x_dim // 2, coordinate[1] - self.y_dim // 2

    def display_board(self):
        """Display the board using matplotlib"""
        colors = []
        x_coords = []
        y_coords = []
        flatArray = self.board.flatten()
        for peg in flatArray:
            if peg is not None:
                colors.append(peg.color)
                x_coords.append(peg.position.x)
                y_coords.append(peg.position.y)
        # Plot the points with their colors
        plt.scatter(x_coords, y_coords, c=colors)
        # Add labels and title
        plt.xlabel('X-axis')
        plt.xticks(list(range(self.x_dim)))
        plt.ylabel('Y-axis')
        plt.yticks(list(range(self.y_dim)))
        plt.title('Checker Board Visualization')
        # Make grid lines
        plt.grid()
        # Show the plot
        plt.show()

    def move_piece(self, player, starting_peg, move_command):
        """Attempt to move a piece for a player"""
        if self.is_valid_move_command(player, starting_peg, move_command):
            pass
        
    def is_valid_move(self, player, starting_peg, move):
        """Check if the move is valid."""
        """"move is a string, that maps to a player direction"""
        direction = player.directions[move]
        target_x = starting_peg.position[0] + direction[0]
        target_y = starting_peg.position[1] + direction[1]
        target_peg = self.board[self.center_coord_to_bot_coord(target_x, target_y)]
        return target_peg.is_empty == True and target_peg.in_board

    def is_valid_move_command(self, player, starting_eg, move_command):
        """"Check if a sequence of commands is valid"""
        pass
        
    def check_winner(self, player):
        """Check if a player has won."""
        pass
        
    def play_game(self):
        """Main game loop."""
        self.display_board()
        current_player = 1
        while True:
            print(f"Player {current_player}'s turn.")
            start = tuple(map(int, input("Enter the piece to move (q, r, s): ").split(',')))
            end = tuple(map(int, input("Enter the destination (q, r, s): ").split(',')))
            if self.move_piece(current_player, start, end):
                self.display_board()
                if self.check_winner(current_player):
                    print(f"Player {current_player} wins!")
                    break
                current_player = 2 if current_player == 1 else 1
            else:
                print("Invalid move. Try again.")

if __name__ == "__main__":
    game = ChineseCheckersBoard(X_DIM, Y_DIM, 6)
    game.display_board()
    # game.play_game()
