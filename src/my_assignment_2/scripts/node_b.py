#!/usr/bin/env python3
import rospy
import sys
from my_assignment_2.srv import GoalsResults, GoalsResultsResponse
# We need to import both, the whole service type and only the response part


# This function will be executed everytime a 'goals_results' service is requeted. It simply gets the value of the
# the parameters 'goals_reached' and 'goals_cancelled' from the ROS parameter server, prints them and returns them.
# The request (req) is blank, since this service has no input arguments 
def serverCallback(req):
	reached = rospy.get_param('goals_reached')
	cancelled = rospy.get_param('goals_cancelled')
	print("\nNumber of goals reached:", reached, "\nNumber of goals cancelled:", cancelled)
	return GoalsResultsResponse(reached, cancelled)
	
	
def main():
	# Initialize a rospy node
	rospy.init_node('goals_results_server')
	
	# Advertise the service 'goals_results', of type 'GoalsResults'
	s = rospy.Service('goals_results', GoalsResults, serverCallback)
	
	print("Service ready! When called, it will provide the number of goals reached and cancelled.")
	
	# Wait forever, continuously checking if a service is requested and, if so, executing the callback function
	rospy.spin() 


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
		
