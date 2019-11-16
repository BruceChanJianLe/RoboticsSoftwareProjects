#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "actionlib_basic/FibonacciAction.h"


class FibonacciAciton
{
    protected:
        // This is a constructor
        // Create a node handle
        ros::NodeHandle nh_;

        // Node Handle must be create before this line. Otherwise strange error occurs
        actionlib::SimpleActionServer<actionlib_basic::FibonacciAction> as_;
        

        std::string action_name_;

        // Create a msg that are used to published feedback and result
        actionlib_basic::FibonacciFeedback feedback_;
        actionlib_basic::FibonacciResult result_;

    public:

        FibonacciAciton(std::string name):
            as_(nh_, name, boost::bind(&FibonacciAciton::executeCB, this, _1), false), action_name_(name)
            {
                as_.start();
            }

        ~FibonacciAciton(void)
        {
        }

        void executeCB(const actionlib_basic::FibonacciGoalConstPtr &goal)
        {
            // Helper variable
            ros::Rate r(1);
            bool success = true;

            // Push_back the seeds for the fibonacci sequence
            feedback_.sequence.clear();
            feedback_.sequence.push_back(0);
            feedback_.sequence.push_back(1);

            // Publish info to the console for the user
            ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(),
             goal->order,
              feedback_.sequence[0],
              feedback_.sequence[1]);
            
            // Start executing the action
            for(int i=1; i <= goal->order; i++)
            {
                // check that preempt has not been requested by the client
                if(as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());

                    // set the action state to preempted
                    as_.setPreempted();

                    // Indicate false
                    success = false;

                    break;
                }

                // Fibonacci addition?
                feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);

                // Publish the feedback
                as_.publishFeedback(feedback_);

                // This sleep is not necessary, the sequence is computed as 1 Hz for demonstartion purpose
                r.sleep();
            }

            if(success)
            {
                result_.sequence = feedback_.sequence;
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                // Set the action state to succeeded
                as_.setSucceeded(result_);
            }
        }
};


int main(int argc, char** argv)
{
    // Initialize ros node
    ros::init(argc, argv, "fibonacci_server");

    // Run the above defined actionlib
    FibonacciAciton fibonacci("fibonacci");

    // To prevent the main loop from exiting, until Ctrl+C or shutdown is called
    ros::spin();

    return 0;
}
