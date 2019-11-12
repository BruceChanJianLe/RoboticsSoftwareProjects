#include "ros/ros.h"
#include "std_msgs/String.h"


// A function that will be run if recieved msgs from publisher
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}


// Main function of the subscriber node
int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "listener");

    // Create a node handler
    ros::NodeHandle n;

    // use the node handler to create a subscriber node and specify the topic name
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // ros::spin() will enter a loop, pumping callbacks. With this version, all
    // callbacks will be called from within this thread (the main one). ros::spin()
    // will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    ros::spin();

}