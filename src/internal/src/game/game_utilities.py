#!/usr/bin/env python
from typing import List
#from checker_map import ChineseCheckersBoard

def convert_list_to_custom_game(num_players: int, players: List[str], curr_player: str, winners: List[str], list_of_positions: List[str]):
        """
        [yellow, ]
        Initializes the board from a custom input which is a list
            input[0]: number of players in the game
            input[1]: players in the game as a list of colors ["Red", 'Gold']
            input[2]: current player, as a color
            input[3]: list of winners as colors
            input[4]: list of lists where each inner list is of the form [x, y, color]

        There are 121 Pegs

        Row lengths: 1, 2, 3, 4, 13, 12, 11, 10, 9, 10, 11, 12, 13, 4, 3, 2, 1
        Height (diff from center): 8, 7, 6, 5, 4, 3, 2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8

        Height: Starts at 16 and ends at 0 
        Each row is centered at 12 with some with a point at the center and others without
        """
        height_diff_to_row_length_dict = {8:1, 7:2, 6:3, 5:4, 4:13, 3:12, 2:11, 1:10, 0:9}
        center_col = 12
        center_row = 8
        converted_points = []
        sliceStart = 0

        # Returns the number of elements used
        def helper(height, sliceStart) -> int:
            # If the heightDiff is even then the center is included
            heightDiff = abs(height - center_row)
            rowLength = height_diff_to_row_length_dict[heightDiff]
            slice = list_of_positions[sliceStart: sliceStart + rowLength]
            halfLength = rowLength // 2
            # center included
            if heightDiff % 2 == 0:
                index = 0
                for x in range(center_col - 2 * halfLength, center_col + 2 * halfLength + 1, 2):
                    color = slice[index]
                    converted_points.append([x, height, color])
                    index += 1
            elif heightDiff % 2 == 1:
                index = 0
                halfLength -= 1
                for x in range(center_col - 2 * halfLength - 1, center_col + 2 * halfLength + 2, 2):
                    color = slice[index]
                    converted_points.append([x, height, color])
                    index += 1
                        
            return sliceStart + rowLength
        
        for height in range(16, -1, -1):
            sliceStart = helper(height, sliceStart)

        return [num_players, players, curr_player, winners, converted_points]


# # Tests for the function
# num_players = 2
# players = ["Gold", "Red"]
# winners = []
# current_player = "Gold"

# def test1():
#     peg_colors = ['Red', 'Black', 'Purple', 'Darkorange', 'Black', 'Red', 'Red', 'Red', 'Green', 'Purple', 'Blue', 'Red', 'Gold', 'Gold', 'Black', 'Green', 'Purple', 'Black', 'Darkorange', 'Purple', 'Red', 'Green', 'Darkorange', 'Black', 'Purple', 'Darkorange', 'Red', 'Black', 'Black', 'Red', 'Gold', 'Green', 'Red', 'Green', 'Red', 'Purple', 'Gold', 'Red', 'Purple', 'Darkorange', 'Green', 'Darkorange', 'Black', 'Darkorange', 'Red', 'Green', 'Gold', 'Blue', 'Blue', 'Green', 'Blue', 'Green', 'Green', 'Blue', 'Green', 'Blue', 'Green', 'Black', 'Green', 'Gold', 'Blue', 'Darkorange', 'Blue', 'Black', 'Black', 'Red', 'Blue', 'Darkorange', 'Red', 'Red', 'Green', 'Black', 'Black', 'Purple', 'Gold', 'Purple', 'Purple', 'Red', 'Purple', 'Gold', 'Gold', 'Darkorange', 'Red', 'Green', 'Purple', 'Purple', 'Black', 'Blue', 'Red', 'Green', 'Darkorange', 'Red', 'Gold', 'Darkorange', 'Purple', 'Blue', 'Purple', 'Purple', 'Green', 'Black', 'Blue', 'Gold', 'Red', 'Darkorange', 'Purple', 'Red', 'Gold', 'Blue', 'Red', 'Green', 'Purple', 'Green', 'Purple', 'Green', 'Red', 'Darkorange', 'Black', 'Blue', 'Darkorange', 'Blue', 'Darkorange']
#     custom_board = convert_list_to_custom_game(num_players, players, current_player, winners, peg_colors)
#     game = ChineseCheckersBoard(custom_board)
#     game.display_board()
#     game.display_until_window_close()

# def test2():
#     peg_colors = ['Purple', 'Darkorange', 'Darkorange', 'Black', 'Darkorange', 'Green', 'Purple', 'Black', 'Blue', 'Blue', 'Red', 'Gold', 'Purple', 'Blue', 'Red', 'Red', 'Blue', 'Green', 'Black', 'Black', 'Red', 'Gold', 'Gold', 'Darkorange', 'Purple', 'Red', 'Darkorange', 'Red', 'Darkorange', 'Gold', 'Black', 'Red', 'Gold', 'Black', 'Darkorange', 'Red', 'Purple', 'Black', 'Darkorange', 'Purple', 'Black', 'Darkorange', 'Purple', 'Darkorange', 'Purple', 'Black', 'Gold', 'Darkorange', 'Black', 'Gold', 'Red', 'Purple', 'Green', 'Purple', 'Red', 'Red', 'Darkorange', 'Blue', 'Darkorange', 'Red', 'Gold', 'Darkorange', 'Darkorange', 'Purple', 'Red', 'Black', 'Red', 'Gold', 'Red', 'Purple', 'Red', 'Green', 'Blue', 'Darkorange', 'Red', 'Red', 'Purple', 'Darkorange', 'Black', 'Blue', 'Green', 'Gold', 'Gold', 'Green', 'Gold', 'Purple', 'Black', 'Gold', 'Blue', 'Green', 'Red', 'Blue', 'Green', 'Gold', 'Black', 'Blue', 'Red', 'Darkorange', 'Green', 'Darkorange', 'Green', 'Red', 'Darkorange', 'Blue', 'Black', 'Gold', 'Gold', 'Purple', 'Blue', 'Gold', 'Green', 'Black', 'Blue', 'Purple', 'Red', 'Black', 'Blue', 'Blue', 'Green', 'Black', 'Blue']
#     custom_board = convert_list_to_custom_game(num_players, players, current_player, winners, peg_colors)
#     game = ChineseCheckersBoard(custom_board)
#     game.display_board()
#     game.display_until_window_close()

# def test3():
#     peg_colors = ['Purple', 'Gold', 'Gold', 'Red', 'Blue', 'Green', 'Darkorange', 'Green', 'Gold', 'Darkorange', 'Gold', 'Gold', 'Blue', 'Gold', 'Black', 'Gold', 'Darkorange', 'Gold', 'Purple', 'Blue', 'Black', 'Black', 'Gold', 'Gold', 'Green', 'Darkorange', 'Gold', 'Gold', 'Red', 'Darkorange', 'Darkorange', 'Gold', 'Purple', 'Red', 'Green', 'Purple', 'Blue', 'Purple', 'Black', 'Purple', 'Darkorange', 'Gold', 'Darkorange', 'Gold', 'Red', 'Purple', 'Red', 'Purple', 'Red', 'Darkorange', 'Darkorange', 'Black', 'Purple', 'Gold', 'Red', 'Black', 'Black', 'Blue', 'Red', 'Darkorange', 'Green', 'Red', 'Darkorange', 'Purple', 'Black', 'Red', 'Black', 'Purple', 'Purple', 'Gold', 'Green', 'Gold', 'Green', 'Blue', 'Black', 'Darkorange', 'Blue', 'Red', 'Green', 'Darkorange', 'Black', 'Darkorange', 'Red', 'Darkorange', 'Red', 'Black', 'Blue', 'Purple', 'Darkorange', 'Purple', 'Blue', 'Darkorange', 'Gold', 'Blue', 'Darkorange', 'Gold', 'Red', 'Purple', 'Black', 'Green', 'Purple', 'Red', 'Darkorange', 'Blue', 'Purple', 'Darkorange', 'Green', 'Purple', 'Red', 'Purple', 'Gold', 'Purple', 'Black', 'Darkorange', 'Gold', 'Green', 'Purple', 'Darkorange', 'Gold', 'Green', 'Blue']
#     custom_board = convert_list_to_custom_game(num_players, players, current_player, winners, peg_colors)
#     game = ChineseCheckersBoard(custom_board)
#     game.display_board()
#     game.display_until_window_close()

# if __name__ == "__main__":
#     test1()
#     test2()
#     test3()