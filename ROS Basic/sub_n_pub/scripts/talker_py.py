#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def talker():

    # Initialize node  
    rospy.init_node("talker", anonymous=True)

    # Declare the publisher
    pub = rospy.Publisher("chatter", String, queue_size=10)

    # Set a Sleep rate for talker to rest before the next msg
    rate = rospy.Rate(10) # 10 Hz

    # To be the same as cpp talker
    count = 0

    while not rospy.is_shutdown():
        # Setup the msg
        msg_str = "hello world {}".format(count)

        # Put the msg in buffer (not yet send)
        rospy.loginfo(msg_str)

        # Send the msg
        pub.publish(msg_str)

        # Sleep before the next msg is publish
        rate.sleep()

        # Increment count by 1 to see the diff between msgs
        count += 1

if __name__ == "__main__":

    try:
        talker()
    except rospy.ROSInterruptException():
        pass
