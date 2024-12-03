from checker_map import ChineseCheckersBoard

X_DIM = 25
Y_DIM = 17

if __name__ == "__main__":
    game = ChineseCheckersBoard(X_DIM, Y_DIM, 2)
    print(len(game.valid_moves(game.player_1)))
    # game.display_board()
    # game.play_game()

def test1() -> bool:
    test1_game = ChineseCheckersBoard(X_DIM, Y_DIM, 2)
    