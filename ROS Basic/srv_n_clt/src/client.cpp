#include "ros/ros.h"
#include "srv_n_clt/AddTwoInts.h"
#include <cstdlib>


// The main function
int main(int argc, char** argv)
{
    // ros node init
    ros::init(argc, argv, "add_two_ints_client");

    // Validate if the input is correct
    if(argc != 3)
    {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    // Create ros node handle, which is a empty node which no definition yet
    ros::NodeHandle n;

    // Create ros service client with the node handle, topic name "add_two_ints"
    ros::ServiceClient client = n.serviceClient<srv_n_clt::AddTwoInts>("add_two_ints");

    // Create a srv data class
    srv_n_clt::AddTwoInts srv;

    // Define the value for request in AddTwoInts class
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    /******************* SIDE NOTES *******************/  

    // atoll(): This function converts a C-type string, passed as an argument to function call,
    // to a long long integer. It parses the C-string str interpreting its content as an integral number,
    // which is returned as a value of type long long int.

    // The first argument is the number of parameters passed plus one to include the name of the program
    // that was executed to get those process running. Thus, argc is always greater than zero and argv[0]
    // is the name of the executable (including the path) that was run to begin this process. 

    /**************************************************/

    // Let client call for a response with the request given
    if(client.call(srv))
    {
        // As variable srv is passed in as an argument the return will be in the srv.response.sum
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        // Printout error msg
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }

    return 0;
}
