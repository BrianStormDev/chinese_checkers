#!/usr/bin/env python
from point import Point
from peg import Peg
from player import Player
from agent import Agent
from typing import List, Tuple, Set
import numpy as np
import matplotlib.pyplot as plt
import time

# Future Optimizations
# Turn most things into constants rather than have unnecessary calculations
#   Player's endzone / startzone, board initialization
# Player's keep track of their current score since calculating it directly isn't possible
# Implement minimax algorithm better
# Optimize the displaying function so that points that haven't changed aren't redrawn (this is the bottleneck)
# Convert code to C++

X_DIM = 26
Y_DIM = 18
POINT_SIZE = 100
PLOT_DELAY = 0.0001
ALL_PLAYER_COLORS = ['GOLD', "PURPLE", "GREEN", "RED", 'DARKORANGE', "BLUE"]

# Decorator used to measure the speed of a function
def timing_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()  # Record the start time
        result = func(*args, **kwargs)   # Call the original function
        end_time = time.perf_counter()   # Record the end time
        execution_time = end_time - start_time
        print(f"Function '{func.__name__}' executed in {execution_time:.6f} seconds")
        return result  # Return the original function's result
    return wrapper

class ChineseCheckersBoard:
    # Class attributes
    # x_dim, y_dim
    # players: List[Player]
    # current_player: Player
    # board: ndarray[Peg]
    # winning_players: List[Player]
    # fig, ax, scatter: matplotlib variables for plotting

####################################################################################################################################################################
    # Board Setup Functions

    def __init__(self, custom_game_state=None):
        """
        Initialize the game object either from a custom state or from a starting state specified by the user
        """
        self.x_dim, self.y_dim = X_DIM, Y_DIM
        self.initialize_player_objects()

        if custom_game_state:
            self.initialize_custom_board(custom_game_state)
        else:
            num_players = self.initialize_num_players()
            self.players = self.initialize_players(num_players)
            self.current_player = self.players[0]
            self.winning_players = []
            self.board = self.initialize_board()

        self.move_history = [] # Subrat Change

    def initialize_player_objects(self):
        """
        Initialize all of the player objects and all of the useful mappings relating to them
        """
        # The player directions, starting from the top most and going clockwise.
        directions_1 = [Point( 1, -1), Point(-1, -1), Point(-2,  0), Point(-1,  1), Point( 1,  1), Point( 2,  0)]
        directions_2 = [Point(-1, -1), Point(-2,  0), Point(-1,  1), Point( 1,  1), Point( 2,  0), Point( 1, -1)]
        directions_3 = [Point(-2,  0), Point(-1,  1), Point( 1,  1), Point( 2,  0), Point( 1, -1), Point(-1, -1)]
        directions_4 = [Point(-1,  1), Point( 1,  1), Point( 2,  0), Point( 1, -1), Point(-1, -1), Point(-2,  0)]
        directions_5 = [Point( 1,  1), Point( 2,  0), Point( 1, -1), Point(-1, -1), Point(-2,  0), Point(-1,  1)]
        directions_6 = [Point( 2,  0), Point( 1, -1), Point(-1, -1), Point(-2,  0), Point(-1,  1), Point( 1,  1)]

        # Creating the playing objects 
        player_1 = Player(1, ALL_PLAYER_COLORS[0], Point(12, 16), directions_1)
        player_2 = Player(2, ALL_PLAYER_COLORS[1], Point(24, 12), directions_2)
        player_3 = Player(3, ALL_PLAYER_COLORS[2], Point(24,  4), directions_3)
        player_4 = Player(4, ALL_PLAYER_COLORS[3], Point(12,  0), directions_4)
        player_5 = Player(5, ALL_PLAYER_COLORS[4], Point( 0,  4), directions_5)
        player_6 = Player(6, ALL_PLAYER_COLORS[5], Point( 0, 12), directions_6)

        # Useful player mappings, using lambda functions
        self.number_to_player = lambda number: {1: player_1, 2: player_2, 3: player_3, 4: player_4, 5: player_5, 6: player_6}[number]
        self.color_to_player = lambda color: {ALL_PLAYER_COLORS[0]: player_1, ALL_PLAYER_COLORS[1]: player_2, ALL_PLAYER_COLORS[2]: player_3, ALL_PLAYER_COLORS[3]: player_4, ALL_PLAYER_COLORS[4]: player_5, ALL_PLAYER_COLORS[5]: player_6}[color.upper()]
        self.color_to_number = lambda color: {'WHITE': -1, 'BLACK': 0, ALL_PLAYER_COLORS[0]: 1, ALL_PLAYER_COLORS[1]: 2, ALL_PLAYER_COLORS[2]: 3, ALL_PLAYER_COLORS[3]: 4, ALL_PLAYER_COLORS[4]: 5, ALL_PLAYER_COLORS[5]: 6}[color.upper()]
        self.all_players = [player_1, player_2, player_3, player_4, player_5, player_6]

    def initialize_custom_board(self, input: List):
        """
        Initialize the board from a custom input
        input[0]: players in the game, as a list of colors: ["color1", "color2", ...]
        input[1]: current player, as a color: "color1"
        input[2]: list of winners, as colors ["color2"]
        input[3]: piece positions, as a list of lists of the form: [x, y, "color1"]
        """
        # Initialize the players
        player_colors = input[0]
        self.players = [self.color_to_player(color) for color in player_colors]

        # Initialize the current player
        current_player_color = input[1]
        self.current_player = self.color_to_player(current_player_color)

        # Initialize the empty board
        self.board = self.initialize_empty_board()

        # Set the pegs of the non players
        non_players = set(self.all_players) - set(self.players)
        for player in non_players:
            for peg in player.current_pegs:
                self.board[peg.position.x, peg.position.y] = peg

        # Clear the pegs of the players to be overwritten later
        for player in self.players:
            player.current_pegs.clear()

        # Set the list of winners
        winners = input[2]
        self.winning_players = [self.color_to_player(color) for color in winners]

        # Each piece is a tuple [x, y, "color1"]
        piece_positions = input[3]
        for piece in piece_positions:
            # Unpack the piece
            piece_x = piece[0]
            piece_y = piece[1]
            piece_color = piece[2].upper()

            # Convert the piece into a peg 
            peg = Peg(Point(piece_x, piece_y), piece_color, True, False)
            player_of_peg = self.color_to_player(piece_color)

            # Add the peg to the player and update the board at that position
            player_of_peg.current_pegs.append(peg)
            self.board[piece_x, piece_y] = peg

    def initialize_empty_board(self) -> np.ndarray:
        """
        Initialize a board with white pegs in the background and black pegs in the playable region
        """
        # Generate the board
        board = np.ndarray((self.x_dim, self.y_dim), dtype=Peg)

        # Fill the whole board as white pegs that are out of the board
        for i in range(self.x_dim):
            for j in range(self.y_dim):
                board[i, j] = Peg(Point(i, j), "WHITE", False, True)

        # Fill the center board as black pegs that are in the board and empty
        hexagon_origin_x, hexagon_origin_y = 12, 8
        for radii in [0, 2, 4, 6, 8]:
            for x_offset in range(-radii, radii + 1):
                for y_offset in range(-radii, radii + 1):
                    if abs(x_offset) + abs(y_offset) == radii:
                        i = x_offset + hexagon_origin_x
                        j = y_offset + hexagon_origin_y
                        board[i, j] =  Peg(Point(i, j), "BLACK", True, True)

        # Initialize each of the empty corners of the board
        self.initialize_player_corner(board, Point(12, 16), Point( 1, -1), Point(-2,  0))
        self.initialize_player_corner(board, Point(24, 12), Point(-1, -1), Point(-1,  1))
        self.initialize_player_corner(board, Point(24,  4), Point(-2,  0), Point( 1,  1))
        self.initialize_player_corner(board, Point(12,  0), Point(-1,  1), Point( 2,  0))
        self.initialize_player_corner(board, Point( 0,  4), Point( 1,  1), Point( 1, -1))
        self.initialize_player_corner(board, Point( 0, 12), Point( 2,  0), Point(-1, -1))

        return board
    
    def initialize_player_corner(self, board: np.ndarray, point: Point, upper_left: Point, right: Point):
        """
        Initialize a corner of the board to be all black empty pegs
        """
        for i in range(4):
            for j in range(0, i + 1):
                cur_position = point + upper_left * i + right * j
                cur_peg = Peg(cur_position, "BLACK", True, True)
                board[cur_position.x, cur_position.y] = cur_peg

    def initialize_num_players(self) -> int:
        """
        Initialize the number of players
        """
        print("\nInitializing the number of players.")
        number = input("Enter the number of players: ")
        while not number.isnumeric() or int(number) < 1 or int(number) > 6 or int(number) % 2 == 1:
            print("Make sure that you input a number that is 2, 4, or 6.")
            number = input("Enter the number of players: ")
        return int(number)

    def initialize_players(self, num_players: int) -> List[Player]:
        """
        Initialize the players in the game
        """
        print("\nInitializing the players.")
        number_of_players = 1
        list_of_players = []
        while number_of_players < num_players:
            color = input(f"Input the color of Player {number_of_players}: ").upper()
            while color not in ALL_PLAYER_COLORS or self.color_to_player(color) in list_of_players:
                print("\nThe color you inputted is either already a player or not a possible player.")
                color = input(f"Input the color of Player {number_of_players}: ").upper()

            # Adding the player corresponding to the color the user input
            player = self.color_to_player(color)
            list_of_players.append(player)

            # Adding the player corresponding to the opposite color of the user input
            player2 = self.get_opposite_player(player)
            color2 = player2.color
            list_of_players.append(player2)

            print(f"Player {number_of_players} is now color {color} and Player {number_of_players + 1} is now color {color2}.")

            number_of_players += 2 # Update the number_of_players

        print(f"The players are {list_of_players}.")
        return list_of_players
    
    def initialize_board(self):
        """
        Initialize the Board
        """
        print("\nInitializing the board.")
        board = self.initialize_empty_board()

        # Initialize the players for the board
        for player in self.all_players:
            for peg in player.current_pegs:
                board[peg.position.x, peg.position.y] = peg
        return board
    
    def reset_game(self):
        """
        Reset the game
        """
        # Reset the pegs of the players and update those pegs on the board
        for player in self.players:
            player.reset_pegs()
            for peg in player.current_pegs:
                self.board[peg.position.x, peg.position.y] = peg
        self.winning_players.clear() # Reset the list of winning players so that players can make moves
        self.scatter = None # Reset the scatter so that the board can be visualized again

####################################################################################################################################################################
# Board visualization functions

    def display_board(self):
        """
        Display the board using matplotlib and set up interactive mode
        """
        print("Displaying the board...")
        plt.ion()  # Turn on interactive mode

        self.fig, self.ax = plt.subplots()  # Create figure and axes

        def on_mouse_click(event):
            # Ensure the event is within the axes
            if event.inaxes:
                # Transform mouse coordinates to data coordinates
                x, y = self.event_coord_to_board_coord(event)
                if x >= 0 and x < self.x_dim and y >= 0 and y < self.y_dim:
                    print(f"\n{self.board[x, y]}")
            else:
                print(self.output_gamestate())

        # Connect the button press event to the callback function
        self.fig.canvas.mpl_connect("button_press_event", on_mouse_click)

        self.setup_graph_labels()
        self.update_board_visual()
        plt.draw()
        plt.pause(PLOT_DELAY)

    def event_coord_to_board_coord(self, event):
        position = self.ax.transData.inverted().transform((event.x, event.y))
        x = int(round(position[0]))
        y = int(round(position[1]))
        return x, y
    
    @timing_decorator
    def update_board_visual(self):
        """Update the displayed board dynamically"""
        peg_array = self.board.flatten()
        colors = [peg.color for peg in peg_array if peg.color != 'WHITE']

        if not hasattr(self, "scatter") or self.scatter == None:
            x_coords = [peg.position.x for peg in peg_array if peg.color != 'WHITE']
            y_coords = [peg.position.y for peg in peg_array if peg.color != 'WHITE']
            self.positions = list(zip(x_coords, y_coords))
            self.scatter = self.ax.scatter(x_coords, y_coords, c=colors, s = POINT_SIZE)
            self.scatter.set_offsets(self.positions)

        self.scatter.set_facecolor(colors)
        self.ax.figure.canvas.draw_idle()
        plt.pause(PLOT_DELAY)

    def setup_graph_labels(self):
        self.ax.set_xlabel("X-axis")
        self.ax.set_xticks(list(range(self.x_dim)))
        self.ax.set_ylabel("Y-axis")
        self.ax.set_yticks(list(range(self.y_dim)))
        self.ax.set_title("Checker Board Visualization")
        self.ax.grid()

    def display_until_window_close(self):
        """Keep the board displayed until the user closes the window"""
        plt.ioff()
        plt.show()

####################################################################################################################################################################
    # Movement Functions

    def is_valid_move_sawyer(self, x_start: int, y_start: int, x_end: int, y_end: int, player_color: str) -> bool: 
        """
        Checks if a move is valid by checking if there is a move in the list of possible moves of a player that has
        the same starting and ending coordinates of the input
        """
        player = self.color_to_player(player_color)
        start_point = Point(x_start, y_start)
        end_point = Point(x_end, y_end)
        moves = set(map(lambda x: (x[0], x[2]), self.valid_player_moves(player)))
        return (start_point, end_point) in moves

    @timing_decorator
    def valid_player_moves(self, player: Player) -> List[Tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        all_moves = []
        for peg in player.current_pegs: # For each of the pegs of the current player
            all_moves.extend(self.valid_peg_moves(peg, player))
        return all_moves
    
    def point_valid_moves(self, point: Point, player: Player) -> List[Tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        peg = self.peg_at_position(point)
        return self.valid_peg_moves(peg, player) 
    
    def valid_peg_moves(self, peg: Peg, player: Player) -> List[Tuple[Point, str, Point]]:
        assert peg in player.current_pegs, "This peg doesn't belong to this player!"
        """
        Generate a list of valid moves for a singular peg of a player as a list of tuples
        Each tuple contains a starting point, the list of move codes to reach the ending point, and the ending point
        """

        def valid_jumps_from_point(visited_positions: set, move_string: str, current_pos: Point) -> Set[Tuple[Point, str, Point]]:
            """
            visited_positions: indicates all of the points we have visited before
            move_string: indicates the moves made up till this point
            origin_pos: indicates the initial position of the peg
            current_pos: indicates the current peg we are looking at
            """
            # Set containing tuples of the possible moves
            jumps = set()

            for move_code, direction in player.directions.items(): # For each possible direction
                if self.is_valid_move(player, current_pos, direction, "Jump"):
                    jump_move_pos = current_pos + (direction * 2) # Get the jump_move_position

                    # Determine if we stop jumping: If the jump will result in the same position, we don't make the recursive jump
                    if jump_move_pos not in visited_positions: 
                        updated_move_code = move_string + "J" + move_code + " " # Builds onto the move code string
                        jumps.add((origin_pos, updated_move_code[:-1], jump_move_pos)) # We can add a valid move
                        visited_positions.add(jump_move_pos) # Update the visited positions
                        jumps.update(valid_jumps_from_point(visited_positions, updated_move_code, jump_move_pos)) # Add all the other valid moves

            return jumps
        
        moves = set()
        origin_pos = peg.position
        
        for move_code, direction in player.directions.items(): # For each possible direction
             # Get the current position of the peg
            single_move_pos = origin_pos + direction

            # First determine if we can make any moves one step away 
            if self.is_valid_move(player, origin_pos, direction, "Regular"):
                moves.add((origin_pos, move_code, single_move_pos))

            # Second determine if we can make any swaps
            if self.is_valid_move(player, origin_pos, direction, "Swap"):
                moves.add((origin_pos, "S" + move_code, single_move_pos))

            # Finally determine if we can make any jumps
            moves.update(valid_jumps_from_point(set([origin_pos]), '', origin_pos))

        return list(moves)

    def move_piece(self, player: Player, starting_pos: Point, move_command: List[str]) -> bool:
        """
        Attempt to move a piece for a player
        return: If the movement is successful, the board and player will be modified and the function will return True or else it will return False
        """
        current_pos = starting_pos
        for move in move_command:
            # Checks if the move is a jump
            if move[0] == "J":
                actual_move = move[1:]
                direction = player.directions[actual_move]
                if self.is_valid_move(player, current_pos, direction, "Jump"):
                    current_pos += (direction * 2)
                else:
                    return False
                
            # Checks if the move is a swap
            elif move[0] == "S":
                actual_move = move[1:]
                direction = player.directions[actual_move]
                if self.is_valid_move(player, current_pos, direction, "Swap"):
                    current_pos += direction
                else:
                    return False
                
            # Checks if the move is a regular move
            else:
                direction = player.directions[move]
                if self.is_valid_move(player, current_pos, direction, "Regular"):
                    current_pos += direction
                else:
                    return False
                
        # Add an additional check that ensure that if the peg started in the end_zone, it can't go out of it
        if (self.in_end_zone(player, starting_pos) and not self.in_end_zone(player, current_pos)):
            return False
        
        # Swap the pegs
        self.swap_pegs(starting_pos, current_pos)

        return True
    
    def swap_pegs(self, starting_pos: Point, final_pos: Point):
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
        
    def is_valid_move(self, player: Player, starting_pos: Point, direction: Point, move_type: str) -> bool:
        """
        Check if the move is valid.
        starting_pos: A Point indicating the starting point of the peg
        direction: A Point indicating the direction of movement
        return: If the singular move is possible, return True
        """
        # Jump case
        if move_type == "Jump":
            return self.is_valid_jump(player, starting_pos, direction)
        
        # Swap Case
        elif move_type == "Swap":
            return self.is_valid_swap(player, starting_pos, direction)
        
        # Regular Move Case
        elif move_type == "Regular":
            target_pos = starting_pos + direction
            if self.in_end_zone(player, starting_pos):
                return self.is_empty(target_pos) and self.in_playable_region(target_pos) and self.in_end_zone(player, target_pos)
            else:
                return self.is_empty(target_pos) and self.in_playable_region(target_pos)
        else:
            assert False, "The move_type you tried is invalid. The valid types are 'Jump', 'Swap', and 'Regular'."

    def is_valid_jump(self, player: Player, starting_pos: Point, direction: Point) -> bool:
        """
        Checks if a jump is valid
        """
        # Calculate the target position and midpoint
        target_pos = starting_pos + direction * 2
        midpoint = starting_pos + direction

        end_zone_check = True

        if self.in_end_zone(player, starting_pos):
            end_zone_check = self.in_end_zone(player, target_pos)

        # The jump must ensure the final position is empty and in the playable region, the mid point is not empty, and that end_zone rules are followed
        return self.is_empty(target_pos) and self.in_playable_region(target_pos) and not self.is_empty(midpoint) and end_zone_check

    def is_valid_swap(self, player: Player, starting_point: Point, direction: Point) -> bool:
        """
        Checks if a swap between two points is valid for a player
        """
        end_point = starting_point + direction
        # Ensure that the end_point is in the end_zone and the end_zone is full of pegs
        if self.in_end_zone(player, end_point) and self.is_end_zone_full(player):
            # Next ensure that you are swapping with a peg of a different color (and must be in bounds)
            starting_peg = self.peg_at_position(starting_point)
            final_peg = self.peg_at_position(end_point)
            return starting_peg.color != final_peg.color
        return False

    def is_end_zone_full(self, player: Player) -> bool:
        """
        Checks if the end_zone of this player is full
        """
        opposite_player = self.get_opposite_player(player)
        starting_zone_points = opposite_player.starting_zone_points
        for point in starting_zone_points:
            peg = self.peg_at_position(point)
            if peg.is_empty:
                return False
        return True
    
    def in_starting_zone(self, player: Player, point: Point) -> bool:
        """
        Checks if a point is in the starting zone of the player.
        """
        return point in player.starting_zone_points

    def in_end_zone(self, player: Player, point: Point) -> bool:
        """
        Checks if a point is in the end zone of the player.
        """
        opposite_player = self.get_opposite_player(player)
        starting_zone_points = opposite_player.starting_zone_points
        return point in starting_zone_points
    
    def peg_at_position(self, position: Point) -> Peg:
        """Return the Peg located at the position"""
        assert self.in_board_array(position), "The point you tried to index is not in bounds." 
        return self.board[position.x, position.y]
    
    def in_board_array(self, position: Point) -> bool:
        """Return whether or not the current position is in bounds"""
        return position.x < self.x_dim and position.x >= 0 and position.y < self.y_dim and position.y >= 0

    def is_empty(self, position: Point) -> bool:
        """Return whether or not there is Peg located at a certain position"""
        return self.in_board_array(position) and self.peg_at_position(position).is_empty
        
    def in_playable_region(self, position: Point) -> bool:
        """Returns whether or not the current position is in the playable region"""
        return self.in_board_array(position) and self.peg_at_position(position).in_board

####################################################################################################################################################################
    # Agent class Functions

    def make_move(self, move: Tuple[Point, str, Point]): # Subrat Change
        start_point = move[0]
        end_point = move [2]
        
        # Store the move for potential undo
        self.move_history.append((start_point, end_point, self.board[end_point.x, end_point.y]))
        
        # Update the board
        moving_peg = self.board[start_point.x, start_point.y]
        if moving_peg.color not in ALL_PLAYER_COLORS:
            print(f"Invalid peg color detected: {moving_peg.color}")
            return  # Or handle error appropriately
        self.board[end_point.x, end_point.y] = moving_peg
        self.board[start_point.x, start_point.y] = Peg(start_point, "BLACK", True, True)
        
        # Update the peg's position
        moving_peg.position = end_point
        
        # Update the player's peg list
        current_player = self.color_to_player(moving_peg.color)
        current_player.current_pegs.remove(moving_peg)
        current_player.current_pegs.append(moving_peg)

    def undo_move(self): # Subrat Change
        if not self.move_history:
            return

        start_point, end_point, replaced_peg = self.move_history.pop()

        # Restore the board state
        moving_peg = self.board[end_point.x, end_point.y]
        self.board[start_point.x, start_point.y] = moving_peg
        self.board[end_point.x, end_point.y] = replaced_peg

        # Update the peg's position
        moving_peg.position = start_point

        # Skip updating player pegs if the moving peg is "Black"
        if moving_peg.color == "BLACK":
            return

        # Update the player's peg list
        current_player = self.color_to_player(moving_peg.color)
        if current_player:
            current_player.current_pegs.remove(moving_peg)
            current_player.current_pegs.append(moving_peg)

    def is_midgame(self) -> bool: 
        """
        Function for the agent class, determines if the game is in the midgame 
        """
        starting_zone_pegs = sum(1 for peg in self.current_player.current_pegs if self.in_starting_zone(self.current_player, peg.position))
        end_zone_pegs = sum(1 for peg in self.current_player.current_pegs if self.in_end_zone(self.current_player, peg.position))

        # Midgame if fewer than half the pegs are in the starting zone, and less than half are in the end zone
        return starting_zone_pegs < 5 and end_zone_pegs < 5

    def is_endgame(self) -> bool: 
        """
        Function for the agent class, determines if the game is in the endgame 
        """
        end_zone_pegs = sum(1 for peg in self.current_player.current_pegs if self.in_end_zone(self.current_player, peg.position))

        # Endgame if more than half the pegs are in the end zone
        return end_zone_pegs > 5

####################################################################################################################################################################
    # UI Gameplay Functions

    def play_game_UI(self, other_player_function):
        """Display the board using matplotlib with dynamic updates"""
        self.fig, self.ax = plt.subplots()  # Create figure and axes
        buffer = [] # Buffer to store the peg the user wants to move

        def on_mouse_press(event):
            if self.is_game_over(): # If the game is over already
                print(f"The game is over! The order of winning is {self.winning_players}.")
                return

            if not event.inaxes: # If the click (event) is not on the graph
                print(f"\nPlayer {self.current_player.number}/{self.current_player.color}'s turn.")
                print(self.output_gamestate())
                buffer.clear()
                return

            # Transform mouse coordinates to data coordinates and then to a point
            x, y = self.event_coord_to_board_coord(event)
            point = Point(x, y)
                
            if buffer: # If a point has already been pressed, attempt the move

                # Checks that the endpoint is a point that can be reached
                possible_moves = self.point_valid_moves(buffer[0], self.current_player)
                possible_endpoints = [move[2] for move in possible_moves]

                # If the final point is in the possible_endpoints
                if point in possible_endpoints: 
                    self.swap_pegs(buffer[0], point) # Swap the pegs
                    print(f"Peg being moved to point ({point.x}, {point.y}). \n")

                    if self.check_player_won(self.current_player): # Check if someone has won
                        print(f"Player {self.current_player.number}/{self.current_player.color} has won in place {len(self.winning_players) + 1}!")
                        self.winning_players.append(self.current_player)

                    self.current_player = self.get_next_player(self.current_player)

                    # Update the board with the other player whether it be a human, Naive AI, or minimax AI
                    other_player_function()

                    # Clear the buffer for the next player and update the board visualy
                    buffer.clear()
                    self.update_board_visual()
                else:
                    # If the final point is not in the possible_endpoints
                    print("The point you pressed is not a valid spot to move to.")

            else: # If there is nothing in the buffer
                # Ensure the peg trying to be moved is in the list of the player's pegs
                if self.peg_at_position(point) in self.current_player.current_pegs:
                    possible_moves = self.point_valid_moves(point, self.current_player)

                    if len(possible_moves) > 0: # Check that this peg has a possible move:
                        print(f"Selected peg at point ({point.x}, {point.y}).")
                        print("If you want to cancel the current peg you have selected, click outside the graph.")
                        buffer.append(point)
                    else:
                        print("This peg has no spots to which it can go to.")
                else:
                    print("The point you pressed is not a valid peg to move in the current player's pegs.")

        # Calls on_mouse_press when the user clicks on the graph
        self.fig.canvas.mpl_connect("button_press_event", on_mouse_press)  

        self.setup_graph_labels() # Set up the axes of the graph
        self.update_board_visual() # Initial draw
        plt.show() # Plot the board

    def human_UI(self):
        """
        Gets the next move from a human
        """
        pass

    def naive_AI_UI(self):
        """
        Gets the next move from the naive AI and updates the game
        """
        self.update_game(self.naive_AI_update_move())

    def minimax_AI_UI(self):
        """
        Gets the next move from the minimax AI and updates the game
        """
        agent = Agent(self.current_player, self, self.get_opposite_player(self.current_player))
        best_move = agent.get_best_move(max_time=1.0)
        if best_move:
            move = self.format_move_for_update_func(best_move)
            self.update_game(move)
        else:
            print("The AI was unable to find a move in time. ")

####################################################################################################################################################################
    # Terminal Gameplay Functions

    def play_game_terminal(self):
        """Main game loop."""
        self.display_board()
        while not self.is_game_over():
            # Print which player it is and all their possible moves
            print(f"Player {self.current_player.number}/{self.current_player.color}'s turn.\n")

            # Print the possible moves
            print("The possible moves are: ")
            for move in self.valid_player_moves(self.current_player):
                print(self.format_move_for_print_func(move))
            print()

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
            self.update_board_visual()

            # Keep the loop going until someone has won the game
            if self.check_player_won(self.current_player):
                print(f"Player {self.current_player.number}/{self.current_player.color} has won in place {len(self.winners) + 1}!")
                self.winning_players.append(self.current_player)

            # Get the next player
            self.current_player = self.get_next_player(self.current_player)

        # Appends the last place finisher
        self.winning_players.append(self.current_player)
        print(f"The game is over! The order of winning is {self.winning_players}.")

    def get_user_input(self) -> List:
        """
        Prompts the user for the move they want to make.
        Converts the string to an array containing a Point and then a series of move commands
        Only ensures that the input is formatted correctly, NOT that the command is possible
        return: [Point, str, str...]
        """
        print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
        user_input = input("Your Input: ")
        user_input_split = user_input.split(" ")
        while len(user_input_split) < 3 or not user_input_split[0].isnumeric or not user_input_split[1].isnumeric or not self.valid_move_string(user_input_split[2:]):
            print("\nYour input was not correctly formatted, try again.")
            print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
            user_input = input("Your Input: ")
            user_input_split = user_input.split(" ")
        x = int(user_input_split[0])
        y = int(user_input_split[1])
        moves = [move.upper() for move in user_input_split[2:]] # Ensures that all of the moves inputted are converted to uppercase
        return [Point(x, y)] + moves
    
    def valid_move_string(self, moves: List[str]) -> bool:
        """
        Checks if the list of moves is valid
        Every move has to be in the list of all possible player moves
        If there is a standard move, there can only be one standard move
        If there is a jump, every move has to be a jump
        """

        # List of single player moves
        basic_moves = ["UL", "UR", "R", "DR", "DL", "L"]
        jump_moves = ["JUL", "JUR", "JR", "JDR", "JDL", "JL"]
        swap_moves = ["SUL", "SUR", "SR", "SDR", "SDL", "SL"]

        basic_exists = False
        jump_count = 0
        swap_exists = False

        # Checks if all of the moves are even in the list of moves
        for move in moves:
            move = move.upper()
            if move in basic_moves:
                basic_exists = True
            elif move in jump_moves:
                jump_count += 1
            elif move in swap_moves:
                swap_exists = True
            else:
                return False

        # If there is a basic move, ensure that the movelist is only that move
        if basic_exists and len(moves) != 1:
            return False
        
        # If there is a jump move, ensure that all moves are jump moves
        if jump_count != 0 and jump_count != len(moves):
            return False
        
        if swap_exists and len(moves) != 1:
            return False
        
        return True
    
    def format_move_for_print_func(self, move: List) -> str:
        """
        Format's a possible move from self.valid_player_moves into a format that can be input into the terminal
        Ex: x y move_command move_command ...
        """
        move_string = ""
        move_string += str(move[0].x)
        move_string += " "
        move_string += str(move[0].y)
        move_string += " "
        move_string += move[1]
        return move_string

    def check_player_won(self, player: Player) -> bool:
        """Check if a player has won."""
        # For every point in the opposite player's "end_zone", we check if the player's point is in that endzoone
        for peg in player.current_pegs:
            if not self.in_end_zone(player, peg.position):
                return False
        return True
    
    def get_opposite_player(self, player: Player) -> Player:
        """
        Returns the opposite player of the input player.
        The opposite player is (player.number + 2) % 6 + 1
        """
        player_number = player.number
        opposite_player = (player_number + 2) % 6 + 1
        return self.number_to_player(opposite_player)
 
    def get_next_player(self, current_player: Player) -> Player:
        """
        Returns the player after the input player.
        The next player clockwise is (player.number % 6) + 1
        """
        assert not self.is_game_over(), "The game is already over, there is no next player."
        current_player_number = current_player.number
        next_player_number = (current_player_number % 6) + 1
        next_player = self.number_to_player(next_player_number)

        # Converts the self.winners which is a list of colors to a list of players
        while next_player not in self.players or next_player in self.winning_players:
            next_player_number = (next_player_number % 6) + 1
            next_player = self.number_to_player(next_player_number)
        return next_player

####################################################################################################################################################################
    # Functional Code and utility functions

    @timing_decorator
    def update_game(self, move_command: List) -> bool:
        """
        Updates the gamestate based on the player_input
        Returns if the move happened
        moveslist[0]: x-coordinate of peg
        moveslist[1]: y-coordinate of peg
        moveslist[2:]: list of move commands (strings)
        """
        # Unpacking the move_command
        x = move_command[0]
        y = move_command[1]
        move_list = move_command[2:]

        starting_peg = Point(x, y)
        move_succeeded = self.move_piece(self.current_player, starting_peg, move_list) 

        # If the player wins from this move, append that player to the list of winning players
        if (self.check_player_won(self.current_player)):
            self.winning_players.append(self.current_player)  

        # If the game is not over yet and the move happened, change the current player to the next player
        if not self.is_game_over() and move_succeeded:
            self.current_player = self.get_next_player(self.current_player)

        # Return if the move succeeded
        return move_succeeded

    @timing_decorator
    def format_move_for_update_func(self, move) -> List:
        """
        Format's a possible move from self.valid_player_moves into a format that can be input as a function call to update_gaame
        Ex: [x, y, move_command move_command ...]
        """
        formatted_move = []
        formatted_move.append(move[0].x)
        formatted_move.append(move[0].y)
        string_moves = move[1]
        string_list = string_moves.split(" ")
        for command in string_list:
            formatted_move.append(command)
        return formatted_move

    def output_gamestate(self) -> List:
        """
        returns the gamestate which can be used to initialize a custom board
        """
        gamestate = []

        # Appending the color of players
        player_colors = [player.color for player in self.players]
        gamestate.append(player_colors)

        # Append the color of the current player
        current_player = self.current_player
        gamestate.append(current_player.color)
        
        # Append the list of winners
        winning_colors = [player.color for player in self.winning_players]
        gamestate.append(winning_colors)

        # Append the remaining pieces on the board
        pieces = []
        for player in self.players:
            for peg in player.current_pegs:
                peg_position = peg.position
                piece = [peg_position.x, peg_position.y, peg.color]
                pieces.append(piece)
        gamestate.append(pieces)

        return gamestate
    
    def is_game_over(self) -> bool:
        """
        Checks to see if the length of players - length of winning players is difference
        """
        return len(self.winning_players) == len(self.players)

    def calculate_height_change_from_move_code(self, move_code: str):
        """
        Calculates the height change of a specific move
        """
        individual_moves = move_code.split(" ")
        up_moves = 0
        down_moves = 0
        for code in individual_moves:
            if "U" in code:
                up_moves += 1
            elif "D" in code:
                down_moves += 1
        return up_moves - down_moves 

    def convert_list_to_custom_game(players: List[str], curr_player: str, winners: List[str], list_of_colors: List[str]):
        """
        Converts the input to an output that is able to construct a custom game.
        """
        points_list = [(12, 16), (11, 15), (13, 15), (10, 14), (12, 14), (14, 14), (9, 13), (11, 13), (13, 13), (15, 13), (0, 12), (2, 12), (4, 12), (6, 12), (8, 12), (10, 12), (12, 12), (14, 12), (16, 12), (18, 12), (20, 12), (22, 12), (24, 12), (1, 11), (3, 11), (5, 11), (7, 11), (9, 11), (11, 11), (13, 11), (15, 11), (17, 11), (19, 11), (21, 11), (23, 11), (2, 10), (4, 10), (6, 10), (8, 10), (10, 10), (12, 10), (14, 10), (16, 10), (18, 10), (20, 10), (22, 10), (3, 9), (5, 9), (7, 9), (9, 9), (11, 9), (13, 9), (15, 9), (17, 9), (19, 9), (21, 9), (4, 8), (6, 8), (8, 8), (10, 8), (12, 8), (14, 8), (16, 8), (18, 8), (20, 8), (3, 7), (5, 7), (7, 7), (9, 7), (11, 7), (13, 7), (15, 7), (17, 7), (19, 7), (21, 7), (2, 6), (4, 6), (6, 6), (8, 6), (10, 6), (12, 6), (14, 6), (16, 6), (18, 6), (20, 6), (22, 6), (1, 5), (3, 5), (5, 5), (7, 5), (9, 5), (11, 5), (13, 5), (15, 5), (17, 5), (19, 5), (21, 5), (23, 5), (0, 4), (2, 4), (4, 4), (6, 4), (8, 4), (10, 4), (12, 4), (14, 4), (16, 4), (18, 4), (20, 4), (22, 4), (24, 4), (9, 3), (11, 3), (13, 3), (15, 3), (10, 2), (12, 2), (14, 2), (11, 1), (13, 1), (12, 0)]

        converted_points = []
        for index in range(len(points_list)):
            point = points_list[index]
            x = point[0]
            y = point[1]
            color = list_of_colors[index]
            if color in players:
                converted_points.append([x, y, color])
        return [players, curr_player, winners, converted_points]

####################################################################################################################################################################
    # Naive AI Code

    def naive_AI_move(self) -> int:
        """
        Naive algorithm that gets the current player and goes through all of their possible moves, looking
        for the best move which is defined as the move that moves one of their pieces the furthest
        """
        moves = self.valid_player_moves(self.current_player)
        best_move_list = []
        best_move_height = 0

        for move in moves:
            move_code = move[1]
            move_height = self.calculate_height_change_from_move_code(move_code)
            if move_height > best_move_height:
                best_move_list.clear()
                best_move_list.append(move)
                best_move_height = move_height
            elif move_height == best_move_height:
                best_move_list.append(move)

        random_move_index = np.random.randint(len(best_move_list))
        return best_move_list[random_move_index]
    
    def naive_AI_update_move(self):
        """
        Returns the optimal move outputted by the naive algorithm to be used by the update function
        """
        move = self.naive_AI_move()
        return self.format_move_for_update_func(move)
    
    def naive_AI_sawyer_move(self):
        """
        Returns the optimal move outputted by the naive algorithm to be used by the internal package (Sawyer)
        """
        move = self.naive_AI_move()
        start, end = move[0], move[2]
        return start.x, start.y, end.x, end.y

####################################################################################################################################################################
    # Minimax AI code

    def minimax_AI_move(self):
        agent = Agent(self.current_player, self, self.get_opposite_player(self.current_player))
        best_move = agent.get_best_move(max_time=1.0)
        if best_move:
            move = self.format_move_for_update_func(best_move)
            return move
        assert False, "The minimax AI was not able to find a move."
    
    def minimax_AI_update_move(self):
        """
        Returns the optimal move outputted by the minimax AI to be used by the update function
        """
        move = self.minimax_AI_move()
        return self.format_move_for_update_func(move)
    
    def minimax_AI_sawyer_move(self):
        """
        Returns the optimal move outputted by the minimax AI to be used by the internal package (Sawyer)
        """
        move = self.minimax_AI_move()
        start, end = move[0], move[2]
        return start.x, start.y, end.x, end.y

####################################################################################################################################################################
    # Main game loop

    def play_game():
        print("Welcome to Chinese Checkers!")
        print("Select Game Mode:")
        print("1. Play on Terminal")
        print("2. Play on UI with Human")
        print("3. Play on UI with Naive AI")
        print("4. Play on UI with Minimax AI")

        mode = input("Enter the number corresponding to the game mode: ")
        game = ChineseCheckersBoard() 

        if mode == "1":
            print("Starting game on Terminal...")
            game.play_game_terminal()

        elif mode == "2":
            print("Starting game on UI with Humans...")
            game.play_game_UI(game.human_UI)

        elif mode == "3":
            print("Starting game on UI with Naive AI...")
            game.play_game_UI(game.naive_AI_UI)

        elif mode == "4":
            print("Starting game on UI with Minimax AI...")
            game.play_game_UI(game.minimax_AI_UI)

        else:
            print("Invalid input. Please restart the program and select a valid game mode.")

if __name__ == "__main__":
    ChineseCheckersBoard.play_game()