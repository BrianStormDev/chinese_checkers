#!/usr/bin/env python
from typing import List
from chineseCheckersBoard import ChineseCheckersBoard

POINTS_LIST = [(12, 16), (11, 15), (13, 15), (10, 14), (12, 14), (14, 14), (9, 13), (11, 13), (13, 13), (15, 13), (0, 12), (2, 12), (4, 12), (6, 12), (8, 12), (10, 12), (12, 12), (14, 12), (16, 12), (18, 12), (20, 12), (22, 12), (24, 12), (1, 11), (3, 11), (5, 11), (7, 11), (9, 11), (11, 11), (13, 11), (15, 11), (17, 11), (19, 11), (21, 11), (23, 11), (2, 10), (4, 10), (6, 10), (8, 10), (10, 10), (12, 10), (14, 10), (16, 10), (18, 10), (20, 10), (22, 10), (3, 9), (5, 9), (7, 9), (9, 9), (11, 9), (13, 9), (15, 9), (17, 9), (19, 9), (21, 9), (4, 8), (6, 8), (8, 8), (10, 8), (12, 8), (14, 8), (16, 8), (18, 8), (20, 8), (3, 7), (5, 7), (7, 7), (9, 7), (11, 7), (13, 7), (15, 7), (17, 7), (19, 7), (21, 7), (2, 6), (4, 6), (6, 6), (8, 6), (10, 6), (12, 6), (14, 6), (16, 6), (18, 6), (20, 6), (22, 6), (1, 5), (3, 5), (5, 5), (7, 5), (9, 5), (11, 5), (13, 5), (15, 5), (17, 5), (19, 5), (21, 5), (23, 5), (0, 4), (2, 4), (4, 4), (6, 4), (8, 4), (10, 4), (12, 4), (14, 4), (16, 4), (18, 4), (20, 4), (22, 4), (24, 4), (9, 3), (11, 3), (13, 3), (15, 3), (10, 2), (12, 2), (14, 2), (11, 1), (13, 1), (12, 0)]

def convert_list_to_custom_game(players: List[str], curr_player: str, winners: List[str], list_of_colors: List[str]):
    """
    Converts the input to an output that is able to construct a custom game.
    """
    converted_points = []
    for index in range(len(POINTS_LIST)):
        point = POINTS_LIST[index]
        x = point[0]
        y = point[1]
        color = list_of_colors[index]
        if color in players:
            converted_points.append([x, y, color])
    return [players, curr_player, winners, converted_points]

# # A test for the function
# if __name__ == "__main__":
#     players = ["Gold", "Red"]
#     winners = []
#     current_player = "Gold"
#     peg_colors = ['Red', 'Black', 'Purple', 'Darkorange', 'Black', 'Red', 'Red', 'Red', 'Green', 'Purple', 'Blue', 'Red', 'Gold', 'Gold', 'Black', 'Green', 'Purple', 'Black', 'Darkorange', 'Purple', 'Red', 'Green', 'Darkorange', 'Black', 'Purple', 'Darkorange', 'Red', 'Black', 'Black', 'Red', 'Gold', 'Green', 'Red', 'Green', 'Red', 'Purple', 'Gold', 'Red', 'Purple', 'Darkorange', 'Green', 'Darkorange', 'Black', 'Darkorange', 'Red', 'Green', 'Gold', 'Blue', 'Blue', 'Green', 'Blue', 'Green', 'Green', 'Blue', 'Green', 'Blue', 'Green', 'Black', 'Green', 'Gold', 'Blue', 'Darkorange', 'Blue', 'Black', 'Black', 'Red', 'Blue', 'Darkorange', 'Red', 'Red', 'Green', 'Black', 'Black', 'Purple', 'Gold', 'Purple', 'Purple', 'Red', 'Purple', 'Gold', 'Gold', 'Darkorange', 'Red', 'Green', 'Purple', 'Purple', 'Black', 'Blue', 'Red', 'Green', 'Darkorange', 'Red', 'Gold', 'Darkorange', 'Purple', 'Blue', 'Purple', 'Purple', 'Green', 'Black', 'Blue', 'Gold', 'Red', 'Darkorange', 'Purple', 'Red', 'Gold', 'Blue', 'Red', 'Green', 'Purple', 'Green', 'Purple', 'Green', 'Red', 'Darkorange', 'Black', 'Blue', 'Darkorange', 'Blue', 'Darkorange']
#     custom_board = convert_list_to_custom_game(players, current_player, winners, peg_colors)
#     game = ChineseCheckersBoard(custom_board)
#     game.display_board()
#     game.display_until_window_close()