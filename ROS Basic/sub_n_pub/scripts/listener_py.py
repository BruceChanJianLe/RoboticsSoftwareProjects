#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):

    rospy.loginfo("I heard: [{}]".format(data.data))
    # rospy.loginfo(rospy.get_caller_id() + "I heard: [{}]".format(data.data))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique name
    # for our 'listener' node so that multiple listeners can run
    # simultaneously.
    
    # Initialize the node
    rospy.init_node("listener", anonymous=True)

    # Create the subscriber node
    rospy.Subscriber("chatter", String, callback)

    # Spin() simply keeps python from exiting until Ctrl+C is pressed
    rospy.spin()


if __name__ == "__main__":

    try:
        listener()
    except rospy.ROSInterruptException:
        pass