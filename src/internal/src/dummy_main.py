#!/usr/bin/env python
# Used to test if the sawyer_actuation packages works by sending the same message continuously
import rospy
from internal.msg import BoardMove

if __name__ == "__main__":
    # Create the ros node
    rospy.init_node('talker', anonymous=True)

    # Create the publisher to the topic using this message
    pub = rospy.Publisher("game_move", BoardMove, queue_size=10)

    while not rospy.is_shutdown():

        # Create the message
        message = BoardMove(0, 4, 24, 12)

        # Log the message 
        rospy.loginfo(f"Test Worked. Message: {message}.")

        # Publish the message
        pub.publish(message)