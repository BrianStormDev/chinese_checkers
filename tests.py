from checker_map import ChineseCheckersBoard
from Point import Point

def jump_loop_test() -> bool:
    red_positions = [[12, 0, "Red"], [13, 1, "Red"], [15, 3, "Red"], [15, 5, "Red"], [13, 5, "Red"], [13, 3, "Red"]]
    yellow_positions = [[9, 3, "Yellow"]]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, [ChineseCheckersBoard.player_1, ChineseCheckersBoard.player_4], 1, positions])
    s = game.output_gamestate()
    game = ChineseCheckersBoard(s)
    game.display_board()
    #game.display_board()
    #print(len(game.valid_moves(game.player_1)))
    game.play_game_terminal()
    
def swapping_test() -> bool:
    red_positions = [[8, 12, "Red"], [10, 14, "Red"], [11, 15, "Red"], [12, 16, "Red"], [11, 13, "Red"], [12, 14, "Red"], [13, 15, "Red"], [13, 13, "Red"], [14, 14, "Red"], [15, 13, "Red"]]
    yellow_positions = [[9, 13, "Yellow"]]
    positions = red_positions + yellow_positions
    game = ChineseCheckersBoard([2, [ChineseCheckersBoard.player_1, ChineseCheckersBoard.player_4], 1, positions])
    # s = game.output_gamestate()
    # game = ChineseCheckersBoard(s)
    # game.display_board()
    print(game.is_valid_swap(game.player_4, Point(8, 12), Point(9, 13)))

if __name__ == "__main__":
    # jump_loop_test()
    swapping_test()
    # game.display_board()
    # game.play_game()