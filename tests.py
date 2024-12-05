from checker_map import ChineseCheckersBoard

def test1() -> bool:
    pass
    
def test2() -> bool:
    pass

if __name__ == "__main__":
    game = ChineseCheckersBoard([2, [ChineseCheckersBoard.player_1, ChineseCheckersBoard.player_4], [16, 8, "Red"]])
    print(len(game.valid_moves(game.player_1)))
    test2()
    # game.display_board()
    # game.play_game()