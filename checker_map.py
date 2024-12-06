import numpy as np
import matplotlib.pyplot as plt
from Peg import Peg
# from typing import List, Set, Tuple
from Player import Player
from Point import Point

class ChineseCheckersBoard:
    # Class attributes
    x_dim = 25
    y_dim = 17
    # num_players
    # players
    # current_player
    # board

    # Information about players and directions to store for later
    yellow_directions = [Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0)]
    purple_directions = [Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1)]
    green_directions = [Point(-2, 0), Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1)]
    red_directions = [Point(-1, 1), Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0)]
    orange_directions = [Point(1, 1), Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1)]
    blue_directions = [Point(2, 0), Point(1, -1), Point(-1, -1), Point(-2, 0), Point(-1, 1), Point(1, 1)]

    # The next player clockwise is player.number + 1
    # The opposite player is (player.number + 3) % 6
    player_1 = Player(1, "Yellow", Point(12, 16), yellow_directions)
    player_2 = Player(2, "Purple", Point(24, 12), purple_directions)
    player_3 = Player(3, "Green", Point(24, 4), green_directions)
    player_4 = Player(4,"Red", Point(12, 0), red_directions)
    player_5 = Player(5, "Orange", Point(0, 4), orange_directions)
    player_6 = Player(6, "Blue", Point(0, 12), blue_directions)

    # Useful player relations
    number_to_player = {1: player_1, 2: player_2, 3: player_3, 4: player_4, 5: player_5, 6: player_6}
    color_to_player = {"Yellow": player_1, "Purple": player_2, "Green": player_3, "Red": player_4, "Orange": player_5, "Blue": player_6}
    player_colors = ["Yellow", "Purple", "Green", "Red", "Orange", "Blue"]
    all_players = [player_1, player_2, player_3, player_4, player_5, player_6]

    # Useful move lists
    basic_moves = ["UL", "UR", "R", "DR", "DL", "L"]
    jump_moves = ["JUL", "JUR", "JR", "JDR", "JDL", "JL"]
    swap_moves = ["SUL", "SUR", "SR", "SDR", "SDL", "SL"]
    player_moves = basic_moves + jump_moves + swap_moves

    #####################################################################################################################################################################
    # Board Setup Functions
    def __init__(self, game_state=None):
        """
        Initialize the game
        """
        if game_state:
            self.initialize_custom_board(game_state)
        else:
            self.num_players = self.initialize_num_players()
            self.players = self.initialize_players()
            self.current_player = self.players[0]
            print("\nInitializing the board.")
            self.board = self.initialize_board()

    def initialize_custom_board(self, input):
        """
        Initializes the board from a custom input which is a list
        input[0]: number of players in the game
        input[1]: players in the game as a list of colors ["Red", "Yellow"]
        input[2]: current player, as an index in the number of players
        input[3]: list of lists where each inner list is of the form [x, y, color]
        """
        # Initializing the number of players
        self.num_players = input[0]

        # Initializing the players
        player_colors = input[1]
        self.players = [self.color_to_player[color] for color in player_colors]

        # Initializing the current player
        current_player_index = input[2]
        self.current_player = self.players[current_player_index]
        
        # Initializing the empty board
        self.board = self.initialize_empty_board()

        # Set the pegs of the non_players
        non_players = set(self.all_players) - set(self.players)
        for player in non_players:
            player.reset_pegs()
            for peg in player.current_pegs:
                self.board[peg.position.x, peg.position.y] = peg

        # Set the pegs of the players
        for player in self.players:
            player.current_pegs.clear()

        # Each piece is a tuple (x, y, color)
        piece_positions = input[3]
        for piece in piece_positions:
            piece_x = piece[0]
            piece_y = piece[1]
            piece_color = piece[2]
            peg = Peg(Point(piece_x, piece_y), piece_color, True, False)
            player_of_peg= self.color_to_player[piece_color]
            player_of_peg.current_pegs.append(peg)
            self.board[piece_x, piece_y] = peg

    def initialize_empty_board(self):
        """
        Initialize a board with white pegs in the background and black pegs in the playable region
        """
        # Fill the whole board as white pegs
        board = np.ndarray((self.x_dim, self.y_dim), dtype=object)
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                board[i, j] = Peg(Point(i, j), "White", False, True)

        # Initialize the center hexagon for the board
        hexagon_origin = Point(12, 8)
        for radii in [0, 2, 4, 6, 8]:
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        i = x + hexagon_origin.x
                        j = y + hexagon_origin.y
                        board[i, j] =  Peg(Point(i, j), "Black", True, True)
        
        # Initializing the side triangles of the board
        self.initialize_corner(board, Point(12, 16), Point(1, -1), Point(-2, 0))
        self.initialize_corner(board, Point(24, 12), Point(-1, -1), Point(-1, 1))
        self.initialize_corner(board, Point(24, 4), Point(-2, 0), Point(1, 1))
        self.initialize_corner(board, Point(12, 0), Point(-1, 1), Point(2, 0))
        self.initialize_corner(board, Point(0, 4), Point(1, 1), Point(1, -1))
        self.initialize_corner(board, Point(0, 12), Point(2, 0), Point(-1, -1))
        
        return board
    
    def initialize_corner(self, board, point: Point, ul: Point, r: Point):
        """
        Initializes a corner of the board to be all black empty pegs
        """
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = point + ul * i + r * j
                cur_peg = Peg(cur_position, "Black", True, True)
                board[cur_position.x, cur_position.y] = cur_peg

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
        Initializing the players in the game
        """
        print("\nInitializing the players.")
        number_of_players = 1
        list_of_players = []
        while number_of_players < self.num_players:
            color = input(f"Input the color of Player {number_of_players}: ")
            while color not in self.player_colors or self.color_to_player[color] in list_of_players:
                print("\nThe color you inputted is either already a player or not a possible player.")
                color = input(f"Input the color of Player {number_of_players}: ")

            # Adding the player corresponding to the color the user input
            player = self.color_to_player[color]
            list_of_players.append(player)

            # Adding the player corresponding to the opposite color of the user input
            player2 = self.get_opposite_player(player)
            color2 = player2.color
            list_of_players.append(player2)

            print(f"Player {number_of_players} is now color {color} and Player {number_of_players + 1} is now color {color2}.")

            # Change the number_of_players
            number_of_players += 2

        print(f"The players are {list_of_players}.")
        return list_of_players

    def initialize_board(self):
        """
        Initialize a Board
        """
        board = self.initialize_empty_board()

        # Initialize the players for the board
        for player in self.all_players:
            # Reset the pegs of the player
            player.reset_pegs()
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

    #####################################################################################################################################################################
    # Movement Functions

    def valid_player_moves(self, player: Player) -> list[tuple[Point, str, Point]]:
        """
        TESTING PURPOSES
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        all_moves = []

        for peg in player.current_pegs: # For each of the pegs of the current player
            all_moves.extend(self.valid_peg_moves(peg, player))

        return all_moves
    
    def point_valid_moves(self, point: Point, player: Player) -> list[tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        peg = self.peg_at_position(point)
        return self.valid_peg_moves(peg, player)
    
    def valid_peg_moves(self, peg: Peg, player: Player) -> list[tuple[Point, str, Point]]:
        def valid_jumps_from_point(visited_positions: set, move_string: str, origin_pos: Point, current_pos: Point) -> set[tuple[Point, str, Point]]:
            """
            Generate a list of valid moves for a singular peg
            visited_positions: indicates all of the points we have visited before
            move_string: indicates the moves made up till this point
            origin_pos: indicates the initial position of the peg
            current_pos: indicates the current peg we are looking at
            """
            # Set containing tuples of the possible moves
            jumps = set()

            for move_code, direction in player.directions.items(): # For each possible direction
                if self.is_valid_move(player, current_pos, direction, True, False):
                    jump_move_pos = current_pos + (direction * 2) # Get the jump_move_position
                    # Determine if we stop jumping:
                    # 1. If the move_tuple is in moves, we don't make the recursive jump
                    # 2. If the jump will result in the same position, we don't make the recursive jump
                    if jump_move_pos not in visited_positions and (jump_move_pos != origin_pos): 
                        updated_move_code = move_string + "J" + move_code + " " # Builds onto the move code string
                        jumps.add((origin_pos, updated_move_code[:-1], jump_move_pos)) # We can add a valid move
                        visited_positions.add(jump_move_pos) # Update the visited positions
                        jumps.update(valid_jumps_from_point(visited_positions, updated_move_code, origin_pos, jump_move_pos)) # Add all the other valid moves
            return jumps
        
        moves = set()
        
        if peg in player.current_pegs:
            for move_code, direction in player.directions.items(): # For each possible direction
                origin_pos = peg.position # Get the current position of the peg
                single_move_pos = origin_pos + direction

                # First determine if we can make any moves one step away 
                if self.is_valid_move(player, origin_pos, direction, False, False):
                    moves.add((origin_pos, move_code, single_move_pos))

                # Second determine if we can make any swaps
                if self.is_valid_move(player, origin_pos, direction, False, True):
                    moves.add((origin_pos, "S" + move_code, single_move_pos))

                # Finally determine if we can make any jumps
                moves.update(valid_jumps_from_point(set(), '', origin_pos, origin_pos))
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
            # Checks if the move is a jump
            if move[0] == "J":
                actual_move = move[1:]
                direction = player.directions[actual_move]
                if self.is_valid_move(player, current_pos, direction, True, False):
                    current_pos += (direction * 2)
                else:
                    return False
                
            # Checks if the move is a swap
            elif move[0] == "S":
                actual_move = move[1:]
                direction = player.directions[actual_move]
                if self.is_valid_move(player, current_pos, direction, False, True):
                    current_pos += direction
                else:
                    return False
                
            # Checks if the move is a regular move
            else:
                direction = player.directions[move]
                if self.is_valid_move(player, current_pos, direction, False, False):
                    current_pos += direction
                else:
                    return False

        # Add an additional check that ensure that if the peg started in the endzone, it can't go out of it
        if (self.in_endzone(player, starting_pos) and not self.in_endzone(player, current_pos)):
            return False
        
        # Swap the pegs
        self.swap_pegs(starting_pos, current_pos)

        return True
    
    def swap_pegs(self, starting_pos: Point, final_pos: Point) -> None:
        """
        Swaps two pegs, one at starting_pos and the other at final_pos
        """
        # There are references to the peg in the board array and also in the player list
        # Ensure that the board array points to the correct peg and the items in the player list are correct
        # The only thing that changes is the position in the pegs and the self.board
        initial_peg = self.peg_at_position(starting_pos)
        final_peg = self.peg_at_position(final_pos)
        initial_peg.position = final_pos
        final_peg.position = starting_pos
        self.board[final_pos.x, final_pos.y] = initial_peg
        self.board[starting_pos.x, starting_pos.y] = final_peg
        
    def is_valid_move(self, player: Player, starting_pos: Point, direction: Point, is_jump: bool, is_swap: bool) -> bool:
        """
        Check if the move is valid.
        starting_pos: A Point indicating the starting point of the peg
        direction: A Point indicating the direction of movement
        return: If the singular move is possible, return True
        """
        # Jump case
        if is_jump:
            return self.is_valid_jump(player, starting_pos, direction)
        
        # Swap Case
        elif is_swap:
            return self.is_valid_swap(player, starting_pos, direction)
        
        # Regular Move Case
        else:
            target_pos = starting_pos + direction
            if self.in_endzone(player, starting_pos):
                return self.is_empty(target_pos) and self.in_bounds(target_pos) and self.in_endzone(player, target_pos)
            else:
                return self.is_empty(target_pos) and self.in_bounds(target_pos)
    
    def is_valid_jump(self, player: Player, starting_pos: Point, direction: Point) -> bool:
        """
        Checks if a jump is valid
        """
        # Determine if we are allowed to make a jump:
            # 1. Target position is in bounds
            # 2. Target position is empty
            # 3. The adjacent location (one_move_pos) has a piece next to it
        # If we start in the endzone, we have an extra condition:
            # 4. The target position must be in the endzone
        target_pos = starting_pos + direction * 2
        midpoint = starting_pos + direction 
        if self.in_endzone(player, starting_pos):
            return self.is_empty(target_pos) and self.in_bounds(target_pos) and not self.is_empty(midpoint) and self.in_endzone(player, target_pos)
        else:
            return self.is_empty(target_pos) and self.in_bounds(target_pos) and not self.is_empty(midpoint)
    
    def is_valid_swap(self, player: Player, starting_point: Point, direction: Point) -> bool:
        """
        Checks if a swap between two points is valid for a player
        """
        end_point = starting_point + direction
        # First ensure that the end_point is in the endzone (you can't swap out of the endzone!)
        if self.in_endzone(player, end_point) and self.in_bounds(end_point):
            # Second ensure that the endzone is full of pegs
            if self.is_endzone_full(player):
                # Third ensure that you are swapping with a peg of a different color (and must be in bounds)
                starting_peg = self.peg_at_position(starting_point)
                final_peg = self.peg_at_position(end_point)
                if (starting_peg.color != final_peg.color):
                    return True
        return False

    def is_endzone_full(self, player: Player) -> bool:
        """
        Checks if the endzone of this player is full
        """
        opposite_player = self.get_opposite_player(player)
        endzone_points = opposite_player.endzone_points
        for point in endzone_points:
            peg = self.peg_at_position(point)
            if peg.is_empty:
                return False
        return True
    
    def in_endzone(self, player: Player, point: Point) -> bool:
        """
        Checks if a point which contains the peg of a player is in the endzone of that player.
        """
        opposite_player = self.get_opposite_player(player)
        endzone_points = opposite_player.endzone_points
        return point in endzone_points
    
    def peg_at_position(self, position: Point) -> Peg:
        """Return the Peg located at the position"""
        return self.board[position.x][position.y]
    
    def is_empty(self, position: Point) -> bool:
        """Return whether or not there is Peg located at a certain position"""
        return not self.in_bounds(position) or self.peg_at_position(position).is_empty
        
    def in_bounds(self, position: Point) -> bool:
        """Return whether or not the current position is in bounds"""
        # First checks to see if the position is in the array structure before checking if the position is in the playable area
        if position.x < self.x_dim and position.x >= 0 and position.y < self.y_dim and position.y >= 0:
            return self.peg_at_position(position).in_board
        return False
    
    #####################################################################################################################################################################
    # Gameplay Functions

    def play_game_UI(self):
        """Display the board using matplotlib with dynamic updates"""

        fig, ax = plt.subplots()  # Create figure and axes

        def redraw_board():
            """Redraw the board with updated peg positions and colors"""
            colors = []
            x_coords = []
            y_coords = []
            flatArray = self.board.flatten()
            for peg in flatArray:
                colors.append(peg.color)
                x_coords.append(peg.position.x)
                y_coords.append(peg.position.y)
            ax.cla()  # Clear the current axes
            ax.scatter(x_coords, y_coords, c=colors)  # Redraw pegs
            ax.set_xlabel("X-axis")
            ax.set_xticks(list(range(self.x_dim)))
            ax.set_ylabel("Y-axis")
            ax.set_yticks(list(range(self.y_dim)))
            ax.set_title("Checker Board Visualization")
            ax.grid()
            fig.canvas.draw_idle()  # Update the plot

        buffer = []

        def on_mouse_move(event):
            if event.inaxes:  # Ensure the event is within the axes
                # Transform mouse coordinates to data coordinates   
                data_coords = ax.transData.inverted().transform((event.x, event.y))
                x = round(data_coords[0])
                y = round(data_coords[1])

                # If the press is on the actual graph
                if x >= 0 and x < self.x_dim and y >= 0 and y < self.y_dim:
                    point = Point(x, y)

                    # If a point has already been pressed, attempt the move
                    if buffer:

                        # Checks that the endpoint is a point that can be reached
                        possible_moves = self.point_valid_moves(buffer[0], self.current_player)
                        possible_endpoints = [move[2] for move in possible_moves]

                        # If the final point is in the possible_endpoints
                        if point in possible_endpoints: 
                            # Swap the pegs
                            self.swap_pegs(buffer[0], point)
                            print(f"Peg being moved to point ({point.x}, {point.y})")

                            # check if someone has won
                            if self.check_player_won(self.current_player):
                                print(f"Player {self.current_player.number}/{self.current_player.color} has won!")

                            # Get the next player, clear the buffer, and redraw the board    
                            self.current_player = self.get_next_player(self.current_player)
                            buffer.clear()
                            redraw_board()
                        else:
                            # If the final point is not in the possible_endpoints
                            print("The point you pressed is not a valid spot to move to")

                    # If there is nothing in the buffer
                    else:
                        # Ensure the peg trying to be moved is in the list of the player's pegs
                        if self.peg_at_position(point) in self.current_player.current_pegs:
                            possible_moves = self.point_valid_moves(point, self.current_player)

                            # Also check that this peg has a possible move:
                            if len(possible_moves) > 0: 
                                print(f"Selected peg at point ({point.x}, {point.y}).\n")
                                buffer.append(point)
                            else:
                                print("This peg has no spots to which it can go to.")
                        else:
                            print("The point you pressed is not a valid peg to move in the current player's pegs")
            else:
                print(f"\nPlayer {self.current_player.number}/{self.current_player.color}'s turn.")
                print("If you want to cancel the current peg you have selected, click outside the graph.\n")
                print(self.output_gamestate())
                buffer.clear()

        fig.canvas.mpl_connect("button_press_event", on_mouse_move)  # Mouse click event
        redraw_board()  # Initial draw
        plt.show()

    def play_game_terminal(self) -> None:
        """Main game loop."""
        self.display_board()
        while True:
            # Print which player it is and all their possible moves
            print(f"Player {self.current_player.number}/{self.current_player.color}'s turn.\n")

            # Loop until the move is possible
            moveslist = self.get_user_input()
            starting_peg = moveslist[0]
            move_command = moveslist[1:]
            while not self.move_piece(self.current_player, starting_peg, move_command):
                print("The command you input is not possible. \n")
                moveslist = self.get_user_input()
                starting_peg = moveslist[0]
                move_command = moveslist[1:]

            # Display the board after the move has been made
            self.display_board()

            # Keep the loop going until someone has won the game
            if self.check_player_won(self.current_player):
                print(f"Player {self.current_player.number}/{self.current_player.color} has won!")
                break

            # Get the next player
            self.current_player = self.get_next_player(self.current_player)

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
    
    def check_player_won(self, player: Player) -> bool:
        """Check if a player has won."""
        # For every point in the opposite player's "endzone", we check if the player's point is in that endzoone
        for peg in player.current_pegs:
            if not self.in_endzone(player, peg.position):
                return False
        return True
    
    def get_opposite_player(self, player: Player) -> Player:
        """Returns the opposite player of the input player."""
        player_number = player.number
        opposite_player = (player_number + 2) % 6 + 1
        return self.number_to_player[opposite_player]

    def get_next_player(self, current_player: Player) -> Player:
        """Returns the player after the input player."""
        current_player_number = current_player.number
        next_player = (current_player_number % 6) + 1
        while (self.number_to_player[next_player] not in self.players):
            next_player = (next_player % 6) + 1
        return self.number_to_player[next_player]

    #####################################################################################################################################################################
    # FUNCTIONAL CODE
    def update_game(self, moveslist) -> bool:
        """
        Updates the gamestate based on the player_input
        Returns if the move happened
        """
        starting_peg = moveslist[0]
        move_command = moveslist[1:]
        move_happened = self.move_piece(self.current_player, starting_peg, move_command)   
        self.current_player = self.get_next_player(self.current_player)
        return move_happened

    def output_gamestate(self):
        """
        returns the gamestate which can be used to initialize a custom board
        """
        # Appending the number of players
        gamestate = []
        gamestate.append(self.num_players)

        # Appending the color of players
        player_colors = [player.color for player in self.players]
        gamestate.append(player_colors)

        # Append the index of the current player
        current_player = self.current_player
        current_player_index = self.players.index(current_player)
        gamestate.append(current_player_index)
        
        piece_positions = []
        for player in self.players:
            for peg in player.current_pegs:
                peg_position = peg.position
                piece = [peg_position.x, peg_position.y, peg.color]
                piece_positions.append(piece)
        gamestate.append(piece_positions)
        return gamestate

if __name__ == "__main__":
    game = ChineseCheckersBoard()
    #game.play_game_terminal()
    game.play_game_UI()