#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "actionlib_basic/FibonacciAction.h"


int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "fibonacci_client");

    // Create an action client
    // True causes the client to spin its own thread
    actionlib::SimpleActionClient<actionlib_basic::FibonacciAction> ac("fibonacci", true);

    // Printout msg to indicate that client node is ready to start
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer(); // This line will wait for infinite time

    ROS_INFO("Action server started, sending goal.");

    // Send goal to the action server
    actionlib_basic::FibonacciGoal goal;

    // Set goal
    goal.order = 20;

    // Send goal
    ac.sendGoal(goal);

    // Wait for response from action
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before timeout");
    }

    // Exit
    return 0;
    
    
}