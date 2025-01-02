#!/usr/bin/env python
# Main file to be run that combines everything in the internal package and publishes a message to the sawyer actuation package

import rospy
import time
from internal.msg import BoardMove
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv.master_vision import image_to_board
from game import *
from game.chineseCheckersBoard import ChineseCheckersBoard

# Hard coded for 2 playerd, Darkorange and purple, with there being no current winners
players = ["Darkorange", "Purple"]
winners = []
IMAGE_COLLECTION_INTERVAL = 2

def image_collection_callback(msg): 
    global last_image_taken_time, image, peg_colors
    try:
        if can_collect_image and time.time() - last_image_taken_time >= IMAGE_COLLECTION_INTERVAL: 
            # Convert the ROS Image message to a OpenCV2 image
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
            new_board = image_to_board(image)
            if new_board != peg_colors: 
                peg_colors = new_board
            last_image_taken_time = time.time()
    except Exception as e:
        rospy.logerr("Error in callback: %s", str(e))

if __name__ == "__main__":
    rospy.init_node('talker', anonymous=True)

    # Create the publisher to the topic using this message
    pub = rospy.Publisher("game_move", BoardMove, queue_size=10)

    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_collection_callback)
    bridge = CvBridge()
    image = None
    last_image_taken_time = time.time()
    peg_colors = []
    can_collect_image = True

    while not rospy.is_shutdown():
        # Get the current player from the user (as a color)
        current_player = input("Enter the current player as a color: ")

        can_collect_image = False # Stop collecting images for now

        # Create the game object and display the board
        custom_board = ChineseCheckersBoard.convert_list_to_custom_game(players, current_player, winners, peg_colors)
        game = ChineseCheckersBoard(custom_board)
        game.display_board()
        game.display_until_window_close()

        # Ask what the user wants to do.
        choice = input("Enter 1 if you want to make the next move, 2 if you want the naive AI to make the next move, 3 if you want the minimax AI to make the next move, or 4 if a move has already been made in real life: ")
        num = int(choice)

        # User specified input
        if num == 1:
            moveInput = input("Enter the start and end position as x y coordinates, space separated: ")
            moveList = moveInput.split(" ")
            x_start = int(moveList[0])
            y_start = int(moveList[1])
            x_end = int(moveList[2])
            y_end = int(moveList[3])
            if not game.is_valid_move_sawyer(x_start, y_start, x_end, y_end, current_player): 
                can_collect_image = True
                print('The input is not a valid move!')
                continue

        # naive AI input
        elif num == 2:
            x_start, y_start, x_end, y_end = game.naive_AI_sawyer_move()

        # minimax AI input
        elif num == 3:
            x_start, y_start, x_end, y_end = game.minimax_AI_sawyer_move()

        # The case when the user makes an irl move and we want to update the board
        elif num == 4:
            can_collect_image = True
            continue

        # Create the message
        # The message is in internal board coordinates
        message = BoardMove(x_start, y_start, x_end, y_end)

        # Log the message
        rospy.loginfo(f"Test Worked. Message: [{x_start}, {y_start}, {x_end}, {y_end}]")

        # Publish the message
        pub.publish(message)

        # Allow the camera to continue collecting images again
        can_collect_image = True
