import rospy
from game.checker_map import ChineseCheckersBoard
from game.game_utilities import convert_list_to_custom_game
from internal.msg import BoardMove

# Hard coded for 2 players, being Gold and red and the current number of winners is none
num_players = 2
players = ["Gold", "Red"]
winners = []

if __name__ == "__main__":
    # THIS IS THE MAIN FILE THROUGH WHICH EVERYTHING IS RUN
    
    rospy.init_node('talker', anonymous=True)

    # Create the publisher to the topic using this message
    pub = rospy.Publisher("game_move", BoardMove, queue_size=10)

    while not rospy.is_shutdown():
        # Get input from camera

        # Either ask the user to select points or have the model find the points
        # Do the homography
        # Get the list of peg colors

        current_player = input("Enter the current player as a color: ")

        peg_colors = []
        custom_board = convert_list_to_custom_game(num_players, players, current_player, winners, peg_colors)
        game = ChineseCheckersBoard(custom_board)

        # Ask if the user wants to make the next move or let the AI come up witha move
        choice = input("Enter 1 if you want to make the next move and 2 if you want the AI to make the next move: ")
        num = int(choice)

        # Handle how the move is inputted
        if num == 1:
            moveInput = input("Enter the start and end position as x y coordinates, spaced separated: ")
            moveList = moveInput.split(" ")
            x_start = moveList[0]
            y_start = moveList[1]
            x_end = moveList[2]
            y_end = moveList[3]
        
        elif num == 2:
            x_start, y_start, x_end, y_end = game.naive_algorithm_initial_and_final_positions()

        # Construct the message
        # The message are the internal board coordinates
        message = BoardMove(x_start, y_start, x_end, y_end)

        # Publish our string to the topic
        pub.publish(message)