import rospy
from game import *
from game.checker_map import ChineseCheckersBoard
from game.game_utilities import convert_list_to_custom_game

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# from cv.master_vision import image_to_board
# import time
from internal.msg import BoardMove

# # # Hard coded for 2 players, being Gold and red and the current number of winners is none
# num_players = 2
# players = ["Gold", "Red"]
# winners = []
# IMAGE_COLLECTION_INTERVAL = 2

# def image_collection_callback(msg): 
#     global last_image_taken_time, image, peg_colors
#     try:
#         if can_collect_image and time.time() - last_image_taken_time >= IMAGE_COLLECTION_INTERVAL: 
#             # Convert the ROS Image message to a OpenCV2 image
#             image = bridge.imgmsg_to_cv2(msg, "bgr8")
#             new_board = image_to_board(image)
#             if new_board != peg_colors: 
#                 peg_colors = new_board
#             last_image_taken_time = time.time()
#     except Exception as e:
#         rospy.logerr("Error in callback: %s", str(e))

if __name__ == "__main__":
    # # THIS IS THE MAIN FILE THROUGH WHICH EVERYTHING IS RUN
    
    rospy.init_node('talker', anonymous=True)

    # Create the publisher to the topic using this message
    pub = rospy.Publisher("game_move", BoardMove, queue_size=10)

    # image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_collection_callback)
    # bridge = CvBridge()
    # image = None
    # last_image_taken_time = time.time()
    # peg_colors = []
    # can_collect_image = True

    while not rospy.is_shutdown():

        # current_player = input("Enter the current player as a color: ")

        # can_collect_image = False

        # custom_board = convert_list_to_custom_game(num_players, players, current_player, winners, peg_colors)
        # game = ChineseCheckersBoard(custom_board)
        # game.display_board()
        # game.display_until_window_close()

        # # Ask if the user wants to make the next move or let the AI come up witha move
        # choice = input("Enter 1 if you want to make the next move and 2 if you want the AI to make the next move: ")
        # num = int(choice)

        # # Handle how the move is inputted
        # if num == 1:
        #     moveInput = input("Enter the start and end position as x y coordinates, spaced separated: ")
        #     moveList = moveInput.split(" ")
        #     x_start = moveList[0]
        #     y_start = moveList[1]
        #     x_end = moveList[2]
        #     y_end = moveList[3]
        
        # elif num == 2:
        #     x_start, y_start, x_end, y_end = game.naive_algorithm_initial_and_final_positions()

        # # Construct the message
        # # The message are the internal board coordinates
        # message = BoardMove(x_start, y_start, x_end, y_end)

        # Dummy test
        message = BoardMove(11, 7, 2, 2)

        rospy.logerr("Test Worked")

        # Publish our string to the topic
        pub.publish(message)

        can_collect_image = True





