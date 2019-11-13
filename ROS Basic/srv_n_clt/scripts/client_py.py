#!/usr/bin/env python

import rospy
import sys
from srv_n_clt.srv import *


# The main client function
def add_two_ints_client(req):

    # Initialize node, if not you will be unable to use rospy.loginfo
    # and you will lose the similarity with the C++ version(depends on user)
    rospy.init_node("add_two_ints_client", anonymous=True)

    # Owing to the fact that python sys module does not have
    # argc which is the number of input, we use len() to determine the value
    if len(req) == 3:
        
        # If variables number is correct, wait for the corresponding ROS topic
        rospy.wait_for_service("add_two_ints")

        try:
            # Create client, specify the topic name and msg type / msg class
            request = rospy.ServiceProxy("add_two_ints", AddTwoInts)

            # Request for a response and store the value of response in a variable
            response = request(int(req[1]), int(req[2]))

            # Since the response only has one arguement which is sum, call it
            rospy.loginfo("Sum: {}".format(response.sum))
        
        except rospy.ServiceException as e:

            # Throw a similar exception as the C++ version
            # Note that the varable e is not used here unlike the tutorial version
            rospy.logerr("Failed to call service add_two_ints")
            
    else:
        rospy.loginfo("usage: add_two_ints_client X Y")


# Check if this is the main function
if __name__ == "__main__":
    try:
        add_two_ints_client(sys.argv)
    except rospy.ROSInterruptException():
        pass