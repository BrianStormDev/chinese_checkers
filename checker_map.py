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
    player_moves = ["UL", "UR", "R", "DR", "DL", "L"]

    def __init__(self):
        """
        Initialize a Board
        """
        self.num_players = self.initialize_num_players()
        self.players = self.initialize_players()
        self.board = self.initialize_board()

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

            print(f'Player {current_player_number} is now color {color} and Player {current_player_number + 1} is now color {color2}.')
            # Change the current_player_number
            current_player_number += 2
        print(f'The players are {list_of_players}.')
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

        #plt.connect('motion_notify_event', on_mouse_move) # If we want to see everytime the mouse moves
        plt.connect('button_press_event', on_mouse_move)  # If we want to see everytime the mouse is clicked

        # Plot the points with their colors
        ax.scatter(x_coords, y_coords, c=colors)
        ax.set_xlabel('X-axis')
        ax.set_xticks(list(range(self.x_dim)))
        ax.set_ylabel('Y-axis')
        ax.set_yticks(list(range(self.y_dim)))
        ax.set_title('Checker Board Visualization')
        ax.grid()
        plt.show()

    # TODO: Here I can either return a list of strings indicating moves, or a list of points...
    def valid_player_moves(self, player: Player) -> list[tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        # all_moves = set()

        # for peg in player.current_pegs: # For each of the pegs of the current player
        #     all_moves.update(self.valid_peg_moves(peg, player))

        # return list(all_moves)

        all_moves = []

        for peg in player.current_pegs: # For each of the pegs of the current player
            all_moves.extend(self.valid_peg_moves(peg, player))

        return all_moves
    
    def single_peg_valid_player_moves(self, point: Point, player: Player) -> list[tuple[Point, str, Point]]:
        """
        Generate a list of valid moves 
        returns: List of valid moves (a list of tuples, where each element is a tuple containing the start point and end point)
        """
        peg = self.board[point.x, point.y]
        return self.valid_peg_moves(peg, player)
    
    # We should probably include a function that determines the valid moves of a single peg for a player
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
                jump_move_pos = origin_pos + (2 * direction)
                if self.in_bounds(jump_move_pos) and self.is_empty(jump_move_pos) and (not self.is_empty(single_move_pos)):
                    updated_move_code = "J" + move_code
                    moves.add((origin_pos, updated_move_code, jump_move_pos))
                    moves.update(valid_jumps_from_point(updated_move_code, origin_pos, jump_move_pos))
        else:
            print("This peg doesn't belong to this player!")

        return list(moves)

        
    def move_piece(self, player: Player, starting_peg: Point, move_command: list[str]) -> bool:
        """Attempt to move a piece for a player"""
        # Anytime we move a piece, the starting_peg must be assigned the color black
        if self.is_valid_move(player, starting_peg, move_command):
            pass
        
    # TODO Figure out how to incorporate is_valid_move into valid_peg_moves
    # This has to do with figuring out if a particular move is valid
    # Should consider starting_point, direction, and whether or not we are jumping (not the player?)
    # Maybe this should be an inner method? Actually maybe not...
    def is_valid_move(self, starting_pos: Point, direction: Point, player: Player) -> bool:
        """
        Check if the move is valid.
        starting_pos: A Point indicating the starting point of the peg
        direction: A Point indicating the direction of movement
        """
        if direction in player.directions.values:
            target_pos = starting_pos + direction
        if self.is_empty(target_pos) and self.in_bounds(target_pos):
            return True
        else:
            return False
    
    def peg_at_position(self, position: Point) -> Peg:
        """Return the Peg located at the position"""
        return self.board[position.x][position.y]
    
    def is_empty(self, position: Point) -> bool:
        """Return whether or not there is Peg located at a certain position"""
        if self.board[position.x][position.y].color == "Black":
            return True
        else: # If the Peg is any other color it is either out of bounds or occupied
            return False
        #return self.peg_at_position(position).is_empty
        
    def in_bounds(self, position: Point) -> bool:
        """Return whether or not the current position is in bounds"""
        if position.x >= self.x_dim or position.x <= 0 or position.y >= self.y_dim or position.y <= 0:
            return False
        else:   
            return True
        #return self.peg_at_position(position).in_board
        
    def check_winner(self, player: Player) -> bool:
        """Check if a player has won."""
        # For every point in the opposite player's "endzone", we check if the player's point is in that endzoone
        current_player_number = player.number
        opposite_player_number = (current_player_number + 2) % 6 + 1
        opposite_player = self.number_to_player_map[opposite_player_number]
        endzone_points = opposite_player.endzone
        for peg in player.current_pegs:
            if peg.position not in endzone_points:
                return False
        return True
    
    def play_game(self) -> None:
        """Main game loop."""
        self.display_board()
        first_player = self.players[0].number
        current_player_number = first_player
        current_player = self.number_to_player_map(current_player_number)
        while True:
            print(f"Player {current_player_number}/{self.number_to_player_map[current_player_number].color}'s turn.")
            moveslist = self.get_user_input()
            starting_peg = moveslist[0]
            move_command = moveslist[1:]
            while not self.move_piece(current_player, starting_peg, move_command):
                moveslist = self.get_user_input()
                starting_peg = moveslist[0]
                move_command = moveslist[1:]
            if self.check_winner(current_player):
                print(f"Player {current_player_number}/{self.number_to_player_map[current_player_number].color} has won!")
                break
            current_player_number = self.get_next_player(current_player_number)
            current_player = self.number_to_player_map(current_player_number)

    def get_user_input(self):
        """
        Prompts the user for the move they want to make.
        Converts the string to an array containing a Point and then a series of move commands
        return: [Point, str, str...]
        """
        print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
        print("Example: 1 2 UR")
        user_input = input("Your Input: ")
        user_input_split = user_input.split(" ")    
        while len(user_input_split) < 3 or not user_input_split[0].isnumeric or not user_input_split[1].isnumeric or not self.string_only_contains_moves(user_input_split[2:]):
            print("Your input was not correctly formatted, try again.")
            print("Enter the move you want to make as a position x y and then the sequential move commands, all space separated.")
            print("Example: 1 2 UR")
            user_input = input("Your Input: ")
            user_input_split = user_input.split(" ")    
        x = user_input_split[0]
        y = user_input_split[1]
        moves = user_input_split[2:]
        return [Point(x, y)] + moves

    def string_only_contains_moves(self, moves: list[str]) -> bool:
        """
        Checks if a list of string contains only moves a player can do
        """
        for move in moves:
            if move not in self.player_moves:
                return False
        return True

    def get_next_player(self, current_player: int) -> int:
        """Returns the number of the next player from the current player's number."""
        next_player = (current_player % 6) + 1
        while (self.number_to_player_map[next_player] not in self.players):
            next_player = (next_player % 6) + 1
        return next_player

if __name__ == "__main__":
    game = ChineseCheckersBoard()
    print(game.check_winner(game.player_1))
    #game.display_board()
    
    for move in game.valid_player_moves(game.player_1):
        print(move)
    print(len(game.valid_player_moves(game.player_1)))
    game.play_game()
