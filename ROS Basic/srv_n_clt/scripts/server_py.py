#!/usr/bin/env python

import rospy
from srv_n_clt.srv import AddTwoInts, AddTwoIntsResponse


def handle_add_two_ints(req):
    # In order to make the printout similar to cpp version
    # I use rospy.loginfo instead of print to display my msg in the terminal
    rospy.loginfo("request: x={}, y={}".format(req.a, req.b))
    rospy.loginfo("sending back response: [{}]".format(req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    # Initialize ros node
    rospy.init_node("add_two_ints_server", anonymous=True)

    # Create a server node
    server = rospy.Service("add_two_ints", AddTwoInts, handle_add_two_ints)

    # Print out something to indicate that the server is running
    rospy.loginfo("Ready to add two ints.")

    # rospy.spin() is to prevent the function from exiting until Ctrl+C is pressed
    rospy.spin()


# Check if this is the main function
if __name__ == "__main__":
    try:
        add_two_ints_server()
    except rospy.ROSInterruptException:
        pass