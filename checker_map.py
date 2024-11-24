import numpy as np
import matplotlib.pyplot as plt

X_DIM = 27
Y_DIM = 34

class Peg:
    def __init__(self, position, in_board, is_empty, color):
        """
        Initialize a peg.
        is_empty: boolean indicating if the peg is empty
        in_board: boolean indicating if the peg is a slot on the board
        color: string indicating the color of the piece on the peg (e.g., 'red', 'blue')
        position: tuple indicating the position of the peg
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

class Border:
    # Class Variables representing directions
    up_slope  = (1, 1)
    down_slope = (-1, -1)
    horizontal = (2, 0)

    def __init__(self, start_point, end_point, direction):
        self.start_point = start_point
        self.end_point = end_point
        self.direction = direction
    def generate_points(self):
        # Generate the difference in x position
        x_steps = np.arange(self.start_point[0], self.end_point[0] + 1, step = self.direction[0])
        # Generate the difference in y position 
        y_steps = np.arange(self.start_point[1], self.end_point[1] + 1, step = self.direction[1])
        return zip(x_steps, y_steps)

class Player:
    def __init__(self, color, number, directions):
        """
        Initialize a player.
        color: string indicating the color of the player (e.g., 'red', 'blue')
        number: integer indicating the number of the player
        directions: an array of strings representing coordinate movements w0ith respect to each player

        Each player should also have a triangle associated with them 
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

red_directions = [(-1, 1), (1, 1), (2, 0), (1, -1), (-1, -1), (-2, 0)]
orange_directions = [(1, 1), (2, 0), (1, -1), (-1, -1), (-2, 0), (-1, 1)]
blue_directions = [(2, 0), (1, -1), (-1, -1), (-2, 0), (-1, 1), (1, 1)]
yellow_directions = [(1, -1), (-1, -1), (-2, 0), (-1, 1), (1, 1), (2, 0)]
purple_directions = [(-1, -1), (-2, 0), (-1, 1), (1, 1), (2, 0), (1, -1)]
green_directions = [(-2, 0), (-1, 1), (1, 1), (2, 0), (1, -1), (-1, -1)]

class ChineseCheckersHex:
    def __init__(self, x_dim, y_dim, num_players=2):
        """
        Initialize a Board
        x_dim: An integer representing the x dimension of the board
        y_dim: An integer representing the y dimension of the board
        """
        self.num_players = num_players
        self.board = self.initialize_board(x_dim, y_dim)

    def initialize_board(self):
        """
        Initialize a Board
        """
        board = np.ndarray((self.x_dim, self.y_dim), dtype=object)
        for x in range(self.x_dim):
            for y in range(self.y_dim):
                x_prime, y_prime = self.bot_coord_to_center_coord(x, y)
                board[x, y] = Peg(x_prime, y_prime, False, True, "black")
        
        for j in range(-4, 5):
            for i in np.arange(- 8 + abs(j), 9 - abs(j), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "pink")

        for j in range(1, 5):
            for i in np.arange(-9 - abs(j) + 1, -9 + abs(j), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "blue")

        for j in range(1, 5):
            for i in np.arange(9 - abs(j) + 1, 9 + abs(j), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "purple")

        for j in range(-4, 0):
            for i in np.arange(-9 - abs(j) + 1, -9 + abs(j), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "orange")

        for j in range(-4, 0):
            for i in np.arange(9 - abs(j) + 1, 9 + abs(j), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "green")
        
        for j in range(5, 9):
            for i in np.arange(- abs(j - 9) + 1, abs(j - 9), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "yellow")

        for j in range(-8, -4):
            for i in np.arange(- abs(j + 9) + 1, abs(j + 9), 2):
                x_prime, y_prime = self.center_coord_to_bot_coord(i, j)
                board[x_prime, y_prime] = Peg(i, j, True, True, "red")
        return board
        
    def center_coord_to_corner_coord(self, coordinate):
        """
        coordinate: tuple indicating a position w.r.t origin at center of board
        returns: tuple indicating a position w.r.t origin at corner of board
        """
        return coordinate[0] + self.x_dim // 2, coordinate[1] + self.y_dim // 2

    def corner_coord_to_center_coord(self, coordinate):
        """
        coordinate: tuple indicating a position w.r.t origin at corner of board
        returns: tuple indicating a position w.r.t origin at center of board
        """
        return coordinate[0] - self.x_dim // 2, coordinate[1] - self.y_dim // 2

    def display_board(self):
        """Display the board using matplotlib"""
        colors = []
        x_coords = []
        y_coords = []
        flatArray = self.board.flatten()
        for point in flatArray:
            colors.append(point.color)
            x_coords.append(point.x)
            y_coords.append(point.y)
        # Plot the points with their colors
        plt.scatter(x_coords, y_coords, c=colors)
        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Scatter Plot of Points with Colors')
        # Show the plot
        plt.show()

    def move_piece(self, player, starting_peg, move_command):
        """Attempt to move a piece for a player."""
        # if self.is_valid_move_command(player, starting_peg, move_command=):
               
        
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
        
    def check_winner(self, player):
        """Check if a player has won."""
        
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
    game = ChineseCheckersHex(27, 19, players=2)
    game.play_game()