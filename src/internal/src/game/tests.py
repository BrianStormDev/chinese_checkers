#!/usr/bin/env python
from checker_map import ChineseCheckersBoard
import numpy as np

def jump_loop_test() -> None:
    red_positions = [[12, 0, "Red"], [13, 1, "Red"], [15, 3, "Red"], [15, 5, "Red"], [13, 5, "Red"], [13, 3, "Red"]]
    yellow_positions = [[9, 3, "Gold"]]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ["Gold", "Red"], "Red", [], positions])
    # Assuming that moves are counted correctly under the loop, we should have 29
    assert len(game.valid_player_moves(game.player_4)) == 29, "Jump Loop Test Failed!"
    print("Jump Loop Test Passed!")
    
def swapping_test_2p() -> None:
    """
    Determines if we able to make any swaps when we have two players
    """
    red_positions = [[8, 12, "Red"], [10, 14, "Red"], [11, 15, "Red"], [12, 16, "Red"], [11, 13, "Red"], [12, 14, "Red"], [13, 15, "Red"], [13, 13, "Red"], [14, 14, "Red"], [15, 13, "Red"]]
    yellow_positions = [[9, 13, "Gold"]]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ["Gold", "Red"], "Red", [], positions])
    valid_swap_moves = [move for move in game.valid_player_moves(game.player_4) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 3, "Two Player Swap Test Failed!"
    print("Two Player Swap Test Passed!")

def swapping_test_6p() -> None:
    """
    Determines if we are able to make any swaps when we have six players
    """
    red_endzone_positions = [[8, 12, "Red"], [11, 15, "Red"], [11, 13, "Red"], [12, 14, "Red"], [13, 15, "Red"], [13, 13, "Red"]]
    red_remaining_positions = [[5, 9, "Red"], [8, 4, "Red"], [18, 10, "Red"], [18, 6, "Red"]]
    yellow_positions = [[9, 13, "Gold"]] 
    green_positions = [[10, 14, "Green"]]
    blue_positions = [[12, 16, "Blue"]]
    orange_positions = [[14, 14, "Darkorange"]]
    purple_positions = [[15, 13, "Purple"]]
    positions = red_endzone_positions + red_remaining_positions + yellow_positions + green_positions + blue_positions + orange_positions + purple_positions
    game = ChineseCheckersBoard([6, ["Red", "Gold", "Green", "Blue", "Darkorange", "Purple"], "Red", [], positions])
    valid_swap_moves = [move for move in game.valid_player_moves(game.player_4) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 11, "Six Player Swap Test Failed!"
    print("Six Player Swap Test Passed!")

def no_swap_test() -> None:
    """
    We should have no swaps allowed if there is an empty spot in the endzone
    """
    red_positions = [[8, 12, "Red"], [10, 14, "Red"], [11, 15, "Red"], [11, 13, "Red"], [12, 14, "Red"], [13, 15, "Red"], [13, 13, "Red"], [14, 14, "Red"], [15, 13, "Red"]]
    yellow_positions = [[9, 13, "Gold"]]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ["Gold", "Red"], "Red", [], positions])
    valid_swap_moves = [move for move in game.valid_player_moves(game.player_4) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 0, "No Swap Test Failed!"
    print("No Swap Test Passed!")

def win_test() -> None:
    red_positions = [[16, 12, 'Red'], [12, 16, 'Red'], [9, 13, 'Red'], [12, 14, 'Red'], [11, 15, 'Red'], [15, 13, 'Red'], [10, 14, 'Red'], [13, 15, 'Red'], [11, 13, 'Red'], [13, 13, 'Red']]
    yellow_positions = [[7, 11, 'Gold'], [19, 9, 'Gold'], [12, 8, 'Gold'], [13, 5, 'Gold'], [8, 10, 'Gold'], [9, 9, 'Gold'], [17, 11, 'Gold'], [9, 7, 'Gold'], [13, 7, 'Gold'], [5, 9, 'Gold']]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ['Red', 'Gold'], "Red", [], positions])
    player = game.color_to_player["Red"]
    game.update_game([16, 12, 'JUL'])
    assert game.check_player_won(player) == True, "Win Test Failed!"
    print("Win Test Passed!")

def endzone_rule_test() -> None:
    red_positions = [[16, 12, 'Red'], [12, 16, 'Red'], [9, 13, 'Red'], [12, 14, 'Red'], [11, 15, 'Red'], [15, 13, 'Red'], [10, 14, 'Red'], [13, 15, 'Red'], [11, 13, 'Red'], [13, 13, 'Red']]
    yellow_positions = [[7, 11, 'Gold'], [19, 9, 'Gold'], [12, 8, 'Gold'], [13, 5, 'Gold'], [8, 10, 'Gold'], [9, 9, 'Gold'], [17, 11, 'Gold'], [9, 7, 'Gold'], [13, 7, 'Gold'], [5, 9, 'Gold']]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ['Red', 'Gold'], "Red", [], positions])
    assert game.update_game([13, 13, 'DL']) == False, "Endzone Rule Failed!"
    print("Endzone Rule Passed!")

def function_game_loop_experiment_2p():
    red_positions = [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red']]
    yellow_positions = [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold']]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, ['Red', 'Gold'], 'Red', [], positions])
    game.display_board()
    for _ in range(10000):
            moves = game.format_for_update_func_possible_moves(game.current_player)
            if (len(moves) != 0):
                j = np.random.randint(0, len(moves))
                game.update_game(moves[j])
                game.update_board_visual()
            else:
                break
    game.display_until_window_close()

def function_game_loop_experiment_6p():
    custom = [6, ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
    game = ChineseCheckersBoard(custom)
    game.display_board()
    for _ in range(20000):
            moves = game.format_for_update_func_possible_moves(game.current_player)
            if (len(moves) != 0):
                j = np.random.randint(0, len(moves))
                game.update_game(moves[j])
                if _ % 10 == 0:
                    game.update_board_visual()
            else:
                break
    game.display_until_window_close()


# Max amount of moves has been found to be 148 empirically
def maxMovesTest():
    numTrials = 100
    maxMoves = 0
    for trial in range(numTrials):
        custom = [6, ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
        game = ChineseCheckersBoard(custom)
        for _ in range(20000):
                moves = game.format_for_update_func_possible_moves(game.current_player)
                maxMoves = max(maxMoves, len(moves))
                if (len(moves) != 0):
                    j = np.random.randint(0, len(moves))
                    game.update_game(moves[j])
                else:
                    break
        print(f"Trial: {trial}, MaxMoves: {maxMoves}")

def calculateRewardTest():
    custom = [6, ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
    game = ChineseCheckersBoard(custom)
    game.display_board()
    for _ in range(20000):
            moves = game.format_for_update_func_possible_moves(game.current_player)
            if (len(moves) != 0):
                j = np.random.randint(0, len(moves))
                s = game.hei(game.current_player, j)
                print(moves[j])
                print(s)
                game.update_game(moves[j])
                game.update_board_visual()
            else:
                break

def naive_vs_random_2p():
    custom = [2, ['Gold', 'Red'], 'Gold', [], [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red']]]
    game = ChineseCheckersBoard(custom)
    game.display_board()
    for _ in range(20000):
            if _ % 6 == 0 and game.get_current_player().color == 'Gold':
                move = game.naive_algorithm_update_move()
                game.update_game(move)
            else:
                moves = game.format_for_update_func_possible_moves(game.current_player)
                if (len(moves) != 0):
                    j = np.random.randint(0, len(moves))
                    game.update_game(moves[j])
                    if _ % 10 == 0:
                        game.update_board_visual()
                else:
                    break
    game.display_until_window_close()

def naive_vs_random_6p():
    custom = [6, ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
    game = ChineseCheckersBoard(custom)
    game.display_board()
    for _ in range(20000):
            if _ % 6 == 0 and game.get_current_player().color == 'Red':
                move = game.naive_algorithm_update_move()
                game.update_game(move)
            else:
                moves = game.format_for_update_func_possible_moves(game.current_player)
                if (len(moves) != 0):
                    j = np.random.randint(0, len(moves))
                    game.update_game(moves[j])
                    if _ % 10 == 0:
                        game.update_board_visual()
                else:
                    break
    game.display_until_window_close()

def naive_vs_random_6p_3_NAIVE():
    custom = [6, ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
    game = ChineseCheckersBoard(custom)
    game.display_board()
    for _ in range(20000):
            if game.get_current_player().color == 'Gold' or game.get_current_player().color == 'Purple' or game.get_current_player().color == 'Green':
                move = game.naive_algorithm_update_move()
                game.update_game(move)
            else:
                moves = game.format_for_update_func_possible_moves(game.current_player)
                if (len(moves) != 0):
                    j = np.random.randint(0, len(moves))
                    game.update_game(moves[j])
                    if _ % 10 == 0:
                        game.update_board_visual()
                else:
                    break
    game.display_until_window_close()

def convert_list_to_custom_game_test():
    testList = ['Blue', 'Green', 'Purple', 'Gold', 'Darkorange', 'Red', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black', 'Black']
    num_players = 6
    players = ['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange']
    curr_player = 'Red'
    winners = []
    custom = ChineseCheckersBoard.convert_list_to_custom_game(num_players, players, curr_player, winners, testList)
    game = ChineseCheckersBoard(custom)
    game.display_board()
    game.display_until_window_close()


if __name__ == "__main__":
    # jump_loop_test()
    # swapping_test_2p()
    # swapping_test_6p()
    # win_test()
    # endzone_rule_test()
    # function_game_loop_experiment_2p()
    # function_game_loop_experiment_6p()
    # maxMovesTest()
    # calculateRewardTest()
    # naive_vs_random_2p()
    # naive_vs_random_6p()
    # naive_vs_random_6p_3_NAIVE()
    convert_list_to_custom_game_test()
    