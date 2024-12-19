#!/usr/bin/env python
from checker_map import ChineseCheckersBoard
import numpy as np
import random
from checker_map import Agent
from Point import Point

CUSTOM_2P = [['Gold', 'Red'], 'Gold', [], [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red']]]
CUSTOM_6P = [['Gold', 'Red', 'Purple', 'Darkorange', 'Green', 'Blue'], 'Gold', [], [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue']]]

# Need to write better descriptions of the test

def jump_loop_test():
    """
    
    """
    custom = [['Gold', 'Red'], 'Red', [], [[9, 3, 'Gold'], [12, 0, 'Red'], [13, 1, 'Red'], [15, 3, 'Red'], [15, 5, 'Red'], [13, 5, 'Red'], [13, 3, 'Red']]]
    game = ChineseCheckersBoard(custom)
    red_player = game.color_to_player["Red"]
    # Assuming that moves are counted correctly under the loop, we should have 29
    assert len(game.valid_player_moves(red_player)) == 29, "Jump Loop Test Failed!"
    print("Jump Loop Test Passed!")
    
def swapping_test_2p():
    """
    Determines if we able to make any swaps when we have two players
    """
    custom = [['Gold', 'Red'], 'Red', [], [[9, 13, 'Gold'], [8, 12, 'Red'], [10, 14, 'Red'], [11, 15, 'Red'], [12, 16, 'Red'], [11, 13, 'Red'], [12, 14, 'Red'], [13, 15, 'Red'], [13, 13, 'Red'], [14, 14, 'Red'], [15, 13, 'Red']]]
    game = ChineseCheckersBoard(custom)
    red_player = game.color_to_player["Red"]
    valid_swap_moves = [move for move in game.valid_player_moves(red_player) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 3, "Two Player Swap Test Failed!"
    print("Two Player Swap Test Passed!")

def swapping_test_6p():
    """
    Determines if we are able to make any swaps when we have six players
    """
    custom = [['Red', 'Gold', 'Green', 'Blue', 'Darkorange', 'Purple'], 'Red', [], [[8, 12, 'Red'], [11, 15, 'Red'], [11, 13, 'Red'], [12, 14, 'Red'], [13, 15, 'Red'], [13, 13, 'Red'], [5, 9, 'Red'], [8, 4, 'Red'], [18, 10, 'Red'], [18, 6, 'Red'], [9, 13, 'Gold'], [10, 14, 'Green'], [12, 16, 'Blue'], [14, 14, 'Darkorange'], [15, 13, 'Purple']]]
    game = ChineseCheckersBoard(custom)
    valid_swap_moves = [move for move in game.valid_player_moves(game.player_4) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 11, "Six Player Swap Test Failed!"
    print("Six Player Swap Test Passed!")

def no_swap_test():
    """
    We should have no swaps allowed if there is an empty spot in the endzone
    """
    custom = [['Gold', 'Red'], 'Red', [], [[9, 13, 'Gold'], [8, 12, 'Red'], [10, 14, 'Red'], [11, 15, 'Red'], [11, 13, 'Red'], [12, 14, 'Red'], [13, 15, 'Red'], [13, 13, 'Red'], [14, 14, 'Red'], [15, 13, 'Red']]]
    game = ChineseCheckersBoard(custom)
    valid_swap_moves = [move for move in game.valid_player_moves(game.player_4) if move[1][0] == "S"]
    assert len(valid_swap_moves) == 0, "No Swap Test Failed!"
    print("No Swap Test Passed!")

def win_test():
    custom = [['Red', 'Gold'], 'Red', [], [[16, 12, 'Red'], [12, 16, 'Red'], [9, 13, 'Red'], [12, 14, 'Red'], [11, 15, 'Red'], [15, 13, 'Red'], [10, 14, 'Red'], [13, 15, 'Red'], [11, 13, 'Red'], [13, 13, 'Red'], [7, 11, 'Gold'], [19, 9, 'Gold'], [12, 8, 'Gold'], [13, 5, 'Gold'], [8, 10, 'Gold'], [9, 9, 'Gold'], [17, 11, 'Gold'], [9, 7, 'Gold'], [13, 7, 'Gold'], [5, 9, 'Gold']]]
    game = ChineseCheckersBoard(custom)
    red_player = game.color_to_player["Red"]
    game.update_game([16, 12, 'JUL'])
    assert game.check_player_won(red_player) == True, "Win Test Failed!"
    print("Win Test Passed!")

def endzone_rule_test():
    red_positions = [[16, 12, 'Red'], [12, 16, 'Red'], [9, 13, 'Red'], [12, 14, 'Red'], [11, 15, 'Red'], [15, 13, 'Red'], [10, 14, 'Red'], [13, 15, 'Red'], [11, 13, 'Red'], [13, 13, 'Red']]
    yellow_positions = [[7, 11, 'Gold'], [19, 9, 'Gold'], [12, 8, 'Gold'], [13, 5, 'Gold'], [8, 10, 'Gold'], [9, 9, 'Gold'], [17, 11, 'Gold'], [9, 7, 'Gold'], [13, 7, 'Gold'], [5, 9, 'Gold']]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([['Red', 'Gold'], "Red", [], positions])
    assert game.update_game([13, 13, 'DL']) == False, "Endzone Rule Failed!"
    print("Endzone Rule Passed!")

def random_moves_2p():
    red_positions = [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red']]
    yellow_positions = [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold']]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([['Red', 'Gold'], 'Red', [], positions])
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

def random_moves_6p():
    custom = [['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
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

def naive_vs_random_2p():
    custom = [['Gold', 'Red'], 'Gold', [], [[12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red']]]
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
    custom = [['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
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

def max_moves_experiment():
    # Max amount of moves has been found to be 148 empirically
    numTrials = 100
    maxMoves = 0
    for trial in range(numTrials):
        custom = [['Red', 'Gold', 'Green', 'Blue', 'Purple', 'Darkorange'], 'Red', [], [[12, 0, 'Red'], [11, 1, 'Red'], [13, 1, 'Red'], [10, 2, 'Red'], [12, 2, 'Red'], [14, 2, 'Red'], [9, 3, 'Red'], [11, 3, 'Red'], [13, 3, 'Red'], [15, 3, 'Red'], [12, 16, 'Gold'], [13, 15, 'Gold'], [11, 15, 'Gold'], [14, 14, 'Gold'], [12, 14, 'Gold'], [10, 14, 'Gold'], [15, 13, 'Gold'], [13, 13, 'Gold'], [11, 13, 'Gold'], [9, 13, 'Gold'], [24, 4, 'Green'], [22, 4, 'Green'], [23, 5, 'Green'], [20, 4, 'Green'], [21, 5, 'Green'], [22, 6, 'Green'], [18, 4, 'Green'], [19, 5, 'Green'], [20, 6, 'Green'], [21, 7, 'Green'], [0, 12, 'Blue'], [2, 12, 'Blue'], [1, 11, 'Blue'], [4, 12, 'Blue'], [3, 11, 'Blue'], [2, 10, 'Blue'], [6, 12, 'Blue'], [5, 11, 'Blue'], [4, 10, 'Blue'], [3, 9, 'Blue'], [24, 12, 'Purple'], [23, 11, 'Purple'], [22, 12, 'Purple'], [22, 10, 'Purple'], [21, 11, 'Purple'], [20, 12, 'Purple'], [21, 9, 'Purple'], [20, 10, 'Purple'], [19, 11, 'Purple'], [18, 12, 'Purple'], [0, 4, 'Darkorange'], [1, 5, 'Darkorange'], [2, 4, 'Darkorange'], [2, 6, 'Darkorange'], [3, 5, 'Darkorange'], [4, 4, 'Darkorange'], [3, 7, 'Darkorange'], [4, 6, 'Darkorange'], [5, 5, 'Darkorange'], [6, 4, 'Darkorange']]]
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

def ai_performance_test(num_games: int = 3):
    import random
    step_counts = []
    results = []

    # Helper function to get all valid hexagonal points
    def get_hexagon_points(hexagon_origin, max_radius):
        hex_points = []
        for radii in range(0, max_radius + 1, 2):  # Ensure the radius matches the hexagon layers
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        i = x + hexagon_origin.x
                        j = y + hexagon_origin.y
                        hex_points.append((i, j))
        return hex_points

    # Helper function to generate unique positions from valid hexagonal points
    def generate_unique_positions(num_positions: int, color: str, valid_points: list, used_positions: set):
        positions = []
        while len(positions) < num_positions:
            pos = random.choice(valid_points)
            if pos not in used_positions:  # Ensure the position is unique
                positions.append([pos[0], pos[1], color])
                used_positions.add(pos)  # Mark this position as used
        return positions

    # Get hexagonal valid points
    hexagon_origin = Point(12, 8)
    max_radius = 8
    valid_hex_points = get_hexagon_points(hexagon_origin, max_radius)

    for game_num in range(num_games):
        # Use a set to track already-used positions
        used_positions = set()

        # Generate random positions for Red and Gold without overlap
        red_positions = generate_unique_positions(10, "Red", valid_hex_points, used_positions)
        gold_positions = generate_unique_positions(10, "Gold", valid_hex_points, used_positions)
        positions = red_positions + gold_positions

        # Initialize the game with random positions
        game = ChineseCheckersBoard([["Red", "Gold"], "Red", [], positions])

        # Ensure the board is displayed
        game.display_board()

        step_counter = 0
        max_steps = 150  # Avoid infinite loops or very long games

        # Run the game where AI plays both sides
        while not game.is_game_over(0) and step_counter < max_steps:
            current_ai = Agent(game.current_player, game, game.get_opposite_player(game.current_player))
            best_move = current_ai.get_best_move(max_time=1.0)
            if best_move:
                game.make_move(best_move)
                game.update_board_visual()  # Visualize the updated board
            else:
                print(f"No valid moves for {game.current_player.color}")
                break
            
            # Check if current player won
            if game.check_player_won(game.current_player):
                results.append(game.current_player.color)
                break
            
            game.current_player = game.get_next_player(game.current_player)
            step_counter += 1

        # Log the number of steps for this game
        step_counts.append(step_counter)
        if not game.is_game_over(0):
            results.append("Unfinished")

        print(f"Game {game_num + 1}/{num_games} finished in {step_counter} steps.")

    # Print summary
    print("\nAI Performance Test Summary:")
    print(f"Total Games: {num_games}")
    print(f"Average Steps per Game: {sum(step_counts) / num_games:.2f}")
    print(f"Game Results: {results}")

def get_hexagon_points(hexagon_origin, max_radius): #made this function
        hex_points = []
        for radii in range(0, max_radius + 1, 2):
            for x in range(-radii, radii + 1):
                for y in range(-radii, radii + 1):
                    if abs(x) + abs(y) == radii:
                        i = x + hexagon_origin.x
                        j = y + hexagon_origin.y
                        hex_points.append((i, j))
        return hex_points

hexagon_origin = Point(12, 8)
max_radius = 8
valid_hex_points = get_hexagon_points(hexagon_origin, max_radius)

def generate_unique_positions(num_positions: int, color: str, valid_points: list, used_positions: set): #made this function
        positions = []
        while len(positions) < num_positions:
            pos = random.choice(valid_points)
            if pos not in used_positions:  # Ensure the position is unique
                positions.append([pos[0], pos[1], color])
                used_positions.add(pos)  # Mark this position as used
        return positions

def get_fixed_positions(): #made this function
    red_positions = [
        [12, 0, "Red"], [11, 1, "Red"], [13, 1, "Red"], [10, 2, "Red"],
        [12, 2, "Red"], [14, 2, "Red"], [9, 3, "Red"], [11, 3, "Red"],
        [13, 3, "Red"], [15, 3, "Red"]
    ]
    gold_positions = [
        [12, 16, "Gold"], [13, 15, "Gold"], [11, 15, "Gold"], [14, 14, "Gold"],
        [12, 14, "Gold"], [10, 14, "Gold"], [15, 13, "Gold"], [13, 13, "Gold"],
        [11, 13, "Gold"], [9, 13, "Gold"]
    ]
    return red_positions + gold_positions

def run_ai_test(num_games: int = 2) -> float: #made this function
    """
    Run multiple games with fixed starting positions to evaluate AI performance.
    Returns the average performance score over the games.
    """
    total_steps = 0
    total_games_finished = 0

    for _ in range(num_games):
        positions = get_fixed_positions()
        game = ChineseCheckersBoard([2, ["Red", "Gold"], "Red", [], positions])
        game.display_board()

        step_counter = 0
        max_steps = 150  # Limit for testing
        while not game.is_game_over(0) and step_counter < max_steps:
            current_ai = Agent(game.current_player, game, game.get_opposite_player(game.current_player))
            best_move = current_ai.get_best_move(max_time=1.0)
            if best_move:
                game.make_move(best_move)
                game.update_board_visual()
            else:
                break

            if game.check_player_won(game.current_player):
                total_games_finished += 1  # Count finished games
                break

            game.current_player = game.get_next_player(game.current_player)
            step_counter += 1

        total_steps += step_counter

    # Return the average score (higher = better performance)
    return total_games_finished / num_games

def tune_parameters(): #made this function
    """
    Tune the AI parameters to maximize performance in a 2-player game with fixed positions.
    """
    best_parameters = None
    best_performance = float('-inf')

    for distance_weight in [5, 10, 15]:
        for jump_bonus in [2, 5, 10]:
            for opponent_penalty_weight in [5, 8, 12]:
                # Set parameters in Agent class
                Agent.distance_weight = distance_weight
                Agent.jump_bonus = jump_bonus
                Agent.opponent_penalty_weight = opponent_penalty_weight

                # Test AI performance
                performance = run_ai_test(num_games=3)
                if performance > best_performance:
                    best_performance = performance
                    best_parameters = (distance_weight, jump_bonus, opponent_penalty_weight)

    print(f"Best Parameters: {best_parameters}, Performance: {best_performance}")

if __name__ == "__main__":
    jump_loop_test()
    swapping_test_2p()
    swapping_test_6p()
    no_swap_test()
    win_test()
    endzone_rule_test()
    # random_moves_2p()
    # random_moves_6p()
    # maxMovesTest()
    # naive_vs_random_2p()
    # naive_vs_random_6p()
    # tune_parameters()