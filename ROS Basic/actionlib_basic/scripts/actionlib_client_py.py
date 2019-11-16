#!/usr/bin/env python

import rospy
import actionlib
import actionlib_basic.msg


def fibonacci_client():
    # Initialize ros node
    rospy.init_node("fibonacci_client")

    # Create a SimpleActionClient and pass the type of action (FibonacciAction) to the constructor
    # Note that the namespace "ns" is the topic name not the node name
    client = actionlib.SimpleActionClient("fibonacci", actionlib_basic.msg.FibonacciAction)

    # Wait until the action server has started up and topic is ready
    client.wait_for_server()

    # Create a goal to send to server
    goal = actionlib_basic.msg.FibonacciGoal(order=3)

    # Send the goal to the action server and wait for response
    client.send_goal(goal)

    # Wait for result for 30 seconds (to be in accordance with the C++ version)
    finished_before_timeout = client.wait_for_result(rospy.Duration(30.0))

    if(not finished_before_timeout):
        rospy.loginfo("Action did not finish before timeout")
    else:
        if actionlib.GoalStatus.SUCCEEDED == client.get_state():
            rospy.loginfo("Action finished: SSUCCEEDED")


if __name__ == "__main__":
    try:
        fibonacci_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupt before completion")