#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the goal message and the result message
import my_actions.msg

def fibonacci_client():
	# Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS
	rospy.init_node('fibonacci_client')
        
    # Creates the SimpleActionClient, passing the type of the action (FibonacciAction) to the constructor
	client = actionlib.SimpleActionClient('fibonacci', my_actions.msg.FibonacciAction)

    # Waits until the action server has started up and started listening for goals
	rospy.loginfo("Waiting for the action server to start")
	client.wait_for_server()
	rospy.loginfo("Action server up and running")

    # Creates a goal to send to the action server
	goal = my_actions.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server
	client.send_goal(goal)
	rospy.loginfo("Goal sent to the action server")

    # Waits for the server to finish performing the action.
	client.wait_for_result()
	rospy.loginfo("Action finished")

    # Prints out the result of executing the action
	result = client.get_result()  # A FibonacciResult
    
    #print("Result:", ', '.join([str(n) for n in result.sequence]))
    
    

if __name__ == '__main__':
	try:
		fibonacci_client()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
        
