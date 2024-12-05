from checker_map import ChineseCheckersBoard

def jump_loop_test() -> bool:
    red_positions = [[12, 0, "Red"], [13, 1, "Red"], [15, 3, "Red"], [15, 5, "Red"], [13, 5, "Red"], [13, 3, "Red"]]
    game = ChineseCheckersBoard([2, [ChineseCheckersBoard.player_2, ChineseCheckersBoard.player_5], []])
    #game.display_board()
    #print(len(game.valid_moves(game.player_1)))
    
def test2() -> bool:
    pass

if __name__ == "__main__":
    jump_loop_test()
    # game.display_board()
    # game.play_game()