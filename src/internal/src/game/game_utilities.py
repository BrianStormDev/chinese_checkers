#!/usr/bin/env python
from typing import List

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
