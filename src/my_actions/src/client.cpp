#include <ros/ros.h>
// The following two libraries are needed for cancelling the goal when needed
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_actions/FibonacciAction.h>
// Even if the action file was called just Fibonacci.action, the header is automatically named FibonacciAction.h
// The ..Action part is always added automatically
// The FibonacciAction type is composed by the types FibonacciGoal, FibonacciResult and FibonacciFeedback
// The global topic will be of type FibonacciAction, while the goal, result and feeback will be of types 
// FibonacciGoal, FibonacciResult and FibonacciFeedback, respectively

int main (int argc, char **argv)
{
	
  ros::init(argc, argv, "test_fibonacci");

  // create the action client. Notice that we don't have nodeHandle here
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<my_actions::FibonacciAction> ac("fibonacci", true);
  // 'ac' is our action client
  // 'fibonacci' is the name of the action service provided by the action server
  // This is the global topic that contains all the 5 individual topics. They will be called
  // '/fibonacci/goal', '/fibonacci/cancel', '/fibonacci/feedback'...

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  my_actions::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal); // call the action service
  // Unlinke the normal services' call() function, this one is non-blocking, so I will just send that goal, the
  // actionServer will start processing it, and I will carry on executing my code

  // Now, I do actively wait for the action to return, but just during 30 seconds
  // If the actionServer does not return within 30 seconds, I will cancel the goal
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout) // I got a return by the the actionServer within 30 seconds
  {
    // Get the state (info from the topic 'status'), which is of the predefined type SimpleClientGoalState
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    
    // Get the result (info from the topic 'result'), which is of the type FibonacciResult
    my_actions::FibonacciResultConstPtr seq= ac.getResult();
    for (int i = 0; i < goal.order; i++) { // Print the sequence that I received from the actionServer
		std::cout << seq->sequence[i] << " ";
	}
	std::cout << std::endl;
    
  }
  else // 30 seconds expired without a return by the actionServer -> cancel the goal
  {
    ROS_INFO("Action did not finish before the time out.");
    ac.cancelGoal(); 
    ROS_INFO("Goal has been cancelled");
   }

  return 0;
}

// We could also use the feedback, getting it periodically within a loop, and checking it. e.g. I could decide to
// cancel the goal if I don't like the feedback I'm receiving
