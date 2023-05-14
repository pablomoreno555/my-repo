#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from my_assignment_2.msg import PosVel

import sys, select, tty, termios # To monitor the keyboard
import actionlib # Bring in the SimpleActionClient

# Bring in the PlanningAction type, composed by the types PlanningGoal, PlanningResult and PlanningFeedback
from assignment_2_2022.msg import PlanningAction, PlanningGoal

# Since we'll publish within the callback function, we have to define the publisher globally
pub = rospy.Publisher("/pos_vel", PosVel, queue_size=10)


# This function will be called every time a msg is received via the topic /odom, and will publish via the topic
# /pos_vel the robot's x and y position, its linear x velocity and its angular z velocity
def callback_odometry(odometry):

	my_pos_vel = PosVel() # this is the msg we'll publish
	
	# Fill the four fields relying on the data received via the topic /odom
	my_pos_vel.x = odometry.pose.pose.position.x
	my_pos_vel.y = odometry.pose.pose.position.y
	my_pos_vel.vel_x = odometry.twist.twist.linear.x
	my_pos_vel.vel_z = odometry.twist.twist.angular.z
	
	pub.publish(my_pos_vel) # publish
	

def main():

	# Initialize a rospy node so that the SimpleActionClient can publish and subscribe over ROS
	rospy.init_node('planning_client')
	
	# Subscribe to the topic /odom
	rospy.Subscriber("/odom", Odometry, callback_odometry)
	
	old_settings = termios.tcgetattr(sys.stdin) # required for monitoring the keyboard
	
	# Counters for the number of goals reached and cancelled, in total
	n_goals_reached = 0
	n_goals_cancelled = 0
	
	# Initialize the parameters, in the ROS parameter service, related to these counters
	rospy.set_param('goals_reached', n_goals_reached)
	rospy.set_param('goals_cancelled', n_goals_cancelled)

    # Create the SimpleActionClient. The service is called '/reaching_goal' and the type of action is 'PlanningAction'
	client = actionlib.SimpleActionClient('reaching_goal', PlanningAction)

    # Wait until the action server has started up and started listening for goals
	print("Waiting for the action server to start...")
	client.wait_for_server()
	print("Action server up and running!\n")
	
	while(1):
		
		# Get a new target from user input
		x = float(input("Please, enter a new target: x = "))
		y = float(input("y = "))

		# Create a goal to send to the action server
		my_goal_pose = PoseStamped()
		my_goal_pose.pose.position.x = x
		my_goal_pose.pose.position.y = y
		goal = PlanningGoal(target_pose = my_goal_pose)

		# Send the goal to the action server
		client.send_goal(goal)
		print("Goal sent to the action server")
		print("Heading to the goal... Press 'c' to cancel the goal")
		
		# Wait until the goal is reached, while monitoring the keyboard to check if the user wants to cancel the goal
		try:
			tty.setcbreak(sys.stdin.fileno())
			while (1):
				
				# If the goal has been successfully reached (sratus=3), update the parameter related to the number of goals
				# reached and break the loop
				if client.get_state() == 3:
					print("Goal reached!\n")
					n_goals_reached += 1
					rospy.set_param('goals_reached', n_goals_reached)
					break
				
				# If the user presses the 'c' key, cancel the current goal, update the parameter related to the number of goals
				# cancelled and break the loop
				elif select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
					key = sys.stdin.read(1)
					if key == 'c':
						client.cancel_goal()
						print("Goal cancelled\n")
						n_goals_cancelled += 1
						rospy.set_param('goals_cancelled', n_goals_cancelled)
						break
						
		finally:
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
			


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion", file=sys.stderr)
		
