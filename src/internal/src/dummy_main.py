#!/usr/bin/env python
import rospy
from internal.msg import BoardMove

if __name__ == "__main__":
    # # THIS IS THE MAIN FILE THROUGH WHICH EVERYTHING IS RUN
    
    rospy.init_node('talker', anonymous=True)

    # Create the publisher to the topic using this message
    pub = rospy.Publisher("game_move", BoardMove, queue_size=10)

    while not rospy.is_shutdown():

        # Dummy test
        message = BoardMove(0, 4, 24, 12)
        # message = BoardMove(24, 12, 24, 12)

        rospy.loginfo(f"Test Worked. Message: {message}.")

        # Publish our string to the topic
        pub.publish(message)