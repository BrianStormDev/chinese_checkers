import numpy as np
import matplotlib.pyplot as plt
import matplotlib.axes as ax
from Peg import Peg
# from typing import List, Set, Tuple
from Player import Player
from Point import Point

# The key thing to note is that a chinese checkers board has players, not a player having a board
# But each player has a set of pieces, representing what pegs they occupy
# The board associates a triangle as the start for each of the players
# The board associates a triangle as the end for each of the players

class ChineseCheckersBoard:
    # Class attributes
    x_dim = 25
    y_dim = 17

    # Information about players and directions to store for later
    yellow_directions = [Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0)]
    purple_directions = [Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1)]
    green_directions = [Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1)]
    red_directions = [Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0)]
    orange_directions = [Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1)]
    blue_directions = [Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1)]

    # The next player clockwise is player.number + 1
    # The opposite player is (player.number + 3) % 6
    player_1 = Player("Yellow", 1, yellow_directions, Point(12, 16))
    player_2 = Player("Purple", 2, purple_directions, Point(24, 12))
    player_3 = Player("Green", 3, green_directions, Point(24, 4))
    player_4 = Player("Red", 4, red_directions, Point(12, 0))
    player_5 = Player("Orange", 5, orange_directions, Point(0, 4))
    player_6 = Player("Blue", 6, blue_directions, Point(0, 12))

    number_to_player_map = {1: player_1, 2: player_2, 3: player_3, 4: player_4, 5: player_5, 6: player_6}
    color_to_player_map = {"Yellow": player_1, "Purple": player_2, "Green": player_3, "Red": player_4, "Orange": player_5, "Blue": player_6}
    player_colors = ["Yellow", "Purple", "Green", "Red", "Orange", "Blue"]
    all_players = [player_1, player_2, player_3, player_4, player_5, player_6]
    basic_moves = ["UL", "UR", "R", "DR", "DL", "L"]
    jump_moves = ["JUL", "JUR", "JR", "JDR", "JDL", "JL"]
    swap_moves = ["SUL", "SUR", "SR", "SDR", "SDL", "SL"]
    player_moves = basic_moves + swap_moves + swap_moves

    def __init__(self, custom=None):
        """
        Initialize a Board
        """
        if custom:
            self.initialize_custom_board(custom)
        else:
            self.num_players = self.initialize_num_players()
            self.players = self.initialize_players()
            self.board = self.initialize_board()

    def initialize_custom_board(self, input):
        """
        Initializes the board from a custom input which is a list
        input[0]: number of players
        input[1]: players in the game as a list [game.player1, game.player2]
        input[2:]: list of lists where each inner list is of the form [x, y, color]
        """
        self.num_players = input[0]
        self.players = input[1]
        self.non_players = set(self.all_players) - set(self.players)
        remaining_pieces = input[2:]

        # The idea is that any untouchable spaces are white, so we can just make the whole board white, then fill in the gaps
        print("\nInitializing the board.")
        self.board = np.ndarray((self.x_dim, self.y_dim), dtype=object)
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                self.board[i, j] = Peg(Point(i, j), False, True, "white")

        # Initialize the hexagon for the board
        hexagon_origin = Point(12, 8)
        for radii in [0, 2, 4, 6, 8]:
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        self.board[x + hexagon_origin.x, y + hexagon_origin.y] =  Peg(Point(x, y) + hexagon_origin, True, True, "Black")

        # Set the pegs of the non_players
        for player in self.non_players:
            for peg in player.current_pegs:
                self.board[peg.position.x, peg.position.y] = peg

        # Set the pegs of the players
        for player in self.players:
            player.current_pegs.clear()

        # Each piece is a tuple (x, y, color)
        for piece in remaining_pieces:
            piece_x = piece[0]
            piece_y = piece[1]
            piece_color = piece[2]
            new_peg = Peg(Point(piece_x, piece_y), True, False, piece_color)
            player_which_piece_belongs_to = self.color_to_player_map[piece_color]
            player_which_piece_belongs_to.current_pegs.append(new_peg)
            self.board[piece[0], piece[1]] = new_peg

        self.display_board()

    def initialize_num_players(self) -> int:
        """
        Initializing the number of players
        """
        print("\nInitializing the number of players.")
        number = input("Enter the number of players: ")
        while not number.isnumeric() or int(number) < 1 or int(number) > 6 or int(number) % 2 == 1:
            print("Make sure that you input a number that is 2, 4, or 6.")
            number = input("Enter the number of players: ")
        return int(number)

    def initialize_players(self) -> list[Player]:
        """
        Initializing the players in the gam
        """
        print("\nInitializing the players.")
        current_player_number = 1
        list_of_players = []
        while current_player_number < self.num_players:
            color = input(f"Input the color of player {current_player_number}: ")
            while color not in self.player_colors or self.color_to_player_map[color] in list_of_players:
                print("The color you inputted is either already a player or not a possible player.")
                color = input(f"Input the color of player {current_player_number}: ")
            # Adding the player corresponding to the color the user input
            player = self.color_to_player_map[color]
            list_of_players.append(player)

            # Adding the player corresponding to the opposite color of the user input
            player_number = player.number
            player2 = self.number_to_player_map[(player_number + 2) % 6 + 1]
            color2 = player2.color
            list_of_players.append(player2)

            print(f"Player {current_player_number} is now color {color} and Player {current_player_number + 1} is now color {color2}.")
            # Change the current_player_number
            current_player_number += 2
        print(f"The players are {list_of_players}.")
        return list_of_players

    def initialize_board(self):
        """
        Initialize a Board
        """
        # The idea is that any untouchable spaces are white, so we can just make the whole board white, then fill in the gaps
        print("\nInitializing the board.")
        board = np.ndarray((self.x_dim, self.y_dim), dtype=object)
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                board[i, j] = Peg(Point(i, j), False, True, "white")

        # Initialize the hexagon for the board
        hexagon_origin = Point(12, 8)
        for radii in [0, 2, 4, 6, 8]:
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        board[x + hexagon_origin.x, y + hexagon_origin.y] =  Peg(Point(x, y) + hexagon_origin, True, True, "Black")

        # Initialize the players for the board
        for player in self.all_players:
            for peg in player.current_pegs:
                board[peg.position.x, peg.position.y] = peg

        return board

    def display_board(self) -> None:
        """Display the board using matplotlib"""
        colors = []
        x_coords = []
        y_coords = []
        flatArray = self.board.flatten()
        for peg in flatArray:
            colors.append(peg.color)
            x_coords.append(peg.position.x)
            y_coords.append(peg.position.y)

        fig, ax = plt.subplots()  # Create figure and axes

        def on_mouse_move(event):
            if event.inaxes:  # Ensure the event is within the axes
                # Transform mouse coordinates to data coordinates
                data_coords = ax.transData.inverted().transform((event.x, event.y))
                x = round(data_coords[0])
                y = round(data_coords[1])
                if x >= 0 and x < self.x_dim and y >= 0 and y < self.y_dim:
                    print(f"Graph coordinates: ({x}, {y}) | {self.board[x, y]}")

        #plt.connect("motion_notify_event", on_mouse_move) # If we want to see everytime the mouse moves
        plt.connect("button_press_event", on_mouse_move)  # If we want to see everytime the mouse is clicked

        # Plot the points with their colors
        ax.scatter(x_coords, y_coords, c=colors)
        ax.set_xlabel("X-axis")
        ax.set_xticks(list(range(self.x_dim)))
        ax.set_ylabel("Y-axis")
        ax.set_yticks(list(range(self.y_dim)))
        ax.set_title("Checker Board Visualization")
        ax.grid()
        plt.show()

    def valid_player_moves(self, player: Player) -> list[tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        all_moves = []

        for peg in player.current_pegs: # For each of the pegs of the current player
            print(peg.position)
            all_moves.extend(self.valid_peg_moves(peg, player))

        return all_moves
    
    def single_peg_valid_player_moves(self, point: Point, player: Player) -> list[tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        peg = self.board[point.x, point.y]
        return self.valid_peg_moves(peg, player)
    
    def valid_peg_moves(self, peg: Peg, player: Player) -> list[tuple[Point, str, Point]]:
        def valid_jumps_from_point(move_string: str, origin_pos: Point, current_pos: Point) -> set[tuple[Point, str, Point]]:
            """
            Generate a list of valid moves for a singular peg
            move_string: indicates the moves made up till this point
            origin_pos: indicates the initial position of the peg
            current_pos: indicates the current peg we are looking at
            """
            # Set containing tuples of 
            jumps = set()

            for move_code, direction in player.directions.items(): # For each possible direction
                one_move_pos = current_pos + direction # Get the one_move_position 
                jump_move_pos = current_pos + (direction * 2) # Get the jump_move_position
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
                        updated_move_code = move_string + " J" + move_code # Builds onto the move code string
                        jumps.add((origin_pos, updated_move_code, jump_move_pos)) # We can add a valid move
                        jumps.update(valid_jumps_from_point(updated_move_code, origin_pos, jump_move_pos)) # Add all the other valid moves
            
            return jumps
        
        moves = set()
        
        if peg in player.current_pegs:
            # First determine if we can make any moves one step away 
            for move_code, direction in player.directions.items(): # For each possible direction
                origin_pos = peg.position # Get the current position of the peg

                # Determine if the singular movement is possible
                # 1. Target position is in bounds
                # 2. Target position is empty
                single_move_pos = origin_pos + direction
                if self.in_bounds(single_move_pos) and self.is_empty(single_move_pos):
                    moves.add((origin_pos, move_code, single_move_pos))

                # Now figure out if we can make any jump movements
                # 1. Target position is in bounds
                # 2. Target position is empty
                # 3. The adjacent location (one_move_pos) has a piece next to it
                jump_move_pos = origin_pos + (direction * 2)
                if self.in_bounds(jump_move_pos) and self.is_empty(jump_move_pos) and (not self.is_empty(single_move_pos)):
                    updated_move_code = "J" + move_code
                    moves.add((origin_pos, updated_move_code, jump_move_pos))
                    moves.update(valid_jumps_from_point(updated_move_code, origin_pos, jump_move_pos))
        else:
            print("This peg doesn't belong to this player!")
        return list(moves)
        
    def move_piece(self, player: Player, starting_pos: Point, move_command: list[str]) -> bool:
        """
        Attempt to move a piece for a player
        return: If the movement is successful, the board and player will be modified and the function will return True
        """
        current_pos = starting_pos
        for move in move_command:
            if move[0] == "J":
                actual_move = move[1:]
                direction = player.directions[actual_move] * 2
                if self.is_valid_move(player, current_pos, direction, True, False):
                    current_pos += direction
                else:
                    return False
            elif move[0] == "S":
                actual_move = move[1:]
                direction = player.directions[actual_move]
            else:
                direction = player.directions[move]
                if self.is_valid_move(player, current_pos, direction, False, False):
                    current_pos += direction
                else:
                    return False
                
            # Adjust the current and starting pos
            # There are references to the peg in the board array and also in the player list
            # Ensure that the board array points to the correct peg and the items in the player list are correct
            # Must be mutated

            print(player.current_pegs)
            initial_peg = self.peg_at_position(starting_pos)  
            final_peg = self.peg_at_position(current_pos)  
            initial_peg.position = current_pos
            final_peg.position = starting_pos
            self.board[current_pos.x, current_pos.y] = initial_peg
            self.board[starting_pos.x, starting_pos.y] = final_peg
            print(player.current_pegs)

        return True
        
    def is_valid_move(self, player: Player, starting_pos: Point, direction: Point, is_jump: bool, is_swap: bool) -> bool:
        """
        Check if the move is valid.
        starting_pos: A Point indicating the starting point of the peg
        direction: A Point indicating the direction of movement
        return: If the singular move is possible, return True
        """
        target_pos = starting_pos + direction
        if is_jump and is_swap:
            midpoint = starting_pos + direction * (0.5)
        elif is_jump:
            midpoint = starting_pos + direction * (0.5)
            return self.is_empty(target_pos) and self.in_bounds(target_pos) and not self.is_empty(midpoint)
        elif is_swap:
            pass
        else:
            return self.is_empty(target_pos) and self.in_bounds(target_pos)
    
    def is_valid_swap(self, player: Player) -> bool:
        opposite = self.get_opposite_player(player)
    
    def peg_at_position(self, position: Point) -> Peg:
        """Return the Peg located at the position"""
        return self.board[position.x][position.y]
    
    def is_empty(self, position: Point) -> bool:
        """Return whether or not there is Peg located at a certain position"""
        return self.peg_at_position(position).is_empty
        
    def in_bounds(self, position: Point) -> bool:
        """Return whether or not the current position is in bounds"""
        # First checks to see if the position is in the array structure before checking if the position is in the playable area
        if position.x < self.x_dim and position.x >= 0 and position.y < self.y_dim and position.y >= 0:
            return self.peg_at_position(position).in_board
        return False
    
    def play_game(self) -> None:
        """Main game loop."""
        self.display_board()
        first_player = self.players[0]
        current_player = first_player
        while True:
            print(f"Player {current_player.number}/{current_player.color}'s turn.")
            for move in game.valid_player_moves(current_player):
                print(move)
            print()
            moveslist = self.get_user_input()
            starting_peg = moveslist[0]
            move_command = moveslist[1:]
            while not self.move_piece(current_player, starting_peg, move_command):
                print("The command you input is not possible. \n")
                moveslist = self.get_user_input()
                starting_peg = moveslist[0]
                move_command = moveslist[1:]
            self.display_board()    
            if self.check_winner(current_player):
                print(f"Player {current_player.number}/{current_player.color} has won!")
                break
            current_player = self.get_next_player(current_player)

    def get_user_input(self):
        """
        Prompts the user for the move they want to make.
        Converts the string to an array containing a Point and then a series of move commands
        Only ensures that the input is formatted correctly, NOT that the command is possible
        return: [Point, str, str...]
        """
        print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
        print("Example: 1 2 UR")
        user_input = input("Your Input: ")
        user_input_split = user_input.split(" ")    
        while len(user_input_split) < 3 or not user_input_split[0].isnumeric or not user_input_split[1].isnumeric or not self.valid_move_string(user_input_split[2:]):
            print("\nYour input was not correctly formatted, try again.")
            print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
            print("Example: 1 2 UR")
            user_input = input("Your Input: ")
            user_input_split = user_input.split(" ")    
        x = int(user_input_split[0])
        y = int(user_input_split[1])
        moves = user_input_split[2:]
        return [Point(x, y)] + moves

    def valid_move_string(self, moves: list[str]) -> bool:
        """
        Checks if the list of moves is valid
        Every move has to be in the list of all possible player moves
        If there is a standard move, there can only be one standard move
        If there is a jump, every move has to be a jump
        """
        # If a player is prevented from winning because of the presence of an opposing peg in the destination area, 
        # the player is entitled to swap the opposing peg with that of his own peg. This applies for a single hole move.

        basic_exists = False
        jump_count = 0
        swap_exists = False

        # Checks if all of the moves are even in the list of moves
        for move in moves:
            if move not in self.player_moves:
                return False
            if move in self.basic_moves:
                basic_exists = True
            elif move in self.jump_moves:
                jump_count += 1
            elif move in self.swap_moves:
                swap_exists = True

        # If there is a basic move, ensure that the movelist is only that move
        if basic_exists and len(moves) != 1:
            return False
        
        # If there is a jump move, ensure that all moves are jump moves
        if jump_count != 0 and jump_count != len(moves):
            return False
        
        if swap_exists and len(moves) != 1:
            return False
        
        return True
    
    def check_winner(self, player: Player) -> bool:
        """Check if a player has won."""
        # For every point in the opposite player's "endzone", we check if the player's point is in that endzoone
        opposite_player = self.get_opposite_player(player)
        endzone_points = opposite_player.endzone_points
        for peg in player.current_pegs:
            if peg.position not in endzone_points:
                return False
        return True
    
    def get_opposite_player(self, player: Player) -> Player:
        """Returns the opposite player of the input player."""
        player_number = player.number
        opposite_player = (player_number + 2) % 6 + 1
        return self.number_to_player_map[opposite_player]

    def get_next_player(self, current_player: Player) -> Player:
        """Returns the player after the input player."""
        current_player_number = current_player.number
        next_player = (current_player_number % 6) + 1
        while (self.number_to_player_map[next_player] not in self.players):
            next_player = (next_player % 6) + 1
        return self.number_to_player_map[next_player]

if __name__ == "__main__":
    game = ChineseCheckersBoard()
    game.play_game()