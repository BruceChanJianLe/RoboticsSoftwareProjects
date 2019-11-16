#!/usr/bin/env python

import rospy
import actionlib
import actionlib_basic.msg


class FibonacciAction:
    # Create msgs that are used to publish feedback/result
    _feedback = actionlib_basic.msg.FibonacciFeedback()
    _result = actionlib_basic.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            actionlib_basic.msg.FibonacciAction,
            # Note that cb here stands for callback
            execute_cb=self.execute_cb,
            auto_start=False
            )
        self._as.start()

    def execute_cb(self, goal):
        # Helper variables
        r = rospy.Rate(1)
        success = True

        # Append the seeds for the fibonacci sequence
        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        # Publish info to the console for the user
        rospy.loginfo("{}: Executing, createing fibonacci sequence of order {} with seeds {}, {}".format(self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        # Start executing the action
        for i in range(1, goal.order):
            # Check if preempt has not been request by the client
            if self._as.is_preempt_requested():
                rospy.loginfo("{}: Preempted".format(self._action_name))
                self._as.set_preempted()
                success = False
                break
            # Perform action of Fibonacci
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i - 1])

            # Publish feedback
            self._as.publish_feedback(self._feedback)

            # This step is not necessart as the sequence is computed at 1 Hz for demonstration purpose
            r.sleep()

        if success:
            # If success, give the feedback sequence to result sequence
            self._result.sequence = self._feedback.sequence

            # Printout success info
            rospy.loginfo("{}: Succeeded".format(self._action_name))

            # Set as succeeded for action
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    try:
        # rospy.init_node("fibonacci_server", anonymous=True)
        rospy.init_node("fibonacci_server")
        # server = FibonacciAction(rospy.get_name())
        server = FibonacciAction("fibonacci")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass