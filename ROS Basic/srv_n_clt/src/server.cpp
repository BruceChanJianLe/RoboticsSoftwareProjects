#include "ros/ros.h"
#include "srv_n_clt/AddTwoInts.h"


// From my define srv, before the --- is the request, and after --- is response
// This function will be run when the server is being requested by the client
// ld and long int is due to the fact that we are using atoll function
bool add(srv_n_clt::AddTwoInts::Request &req, srv_n_clt::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}


// The main function
int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "add_two_ints_server");

    // Create ros node handle, which has not define it to be anything yet
    ros::NodeHandle n;

    // Define the node as a server/service
    // Note the the node name is "add_two_ints_server" and the topic name is "add_two_ints"
    ros::ServiceServer service = n.advertiseService("add_two_ints", add);

    // Print out something to indicate that the service is running
    ROS_INFO("Ready to add two ints.");

    // ros::spin() to keep the main function from exiting until Ctrl+C is pressed
    ros::spin();

    return 0;
}