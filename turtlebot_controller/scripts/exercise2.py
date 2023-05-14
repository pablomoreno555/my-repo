#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
import time

x = -1
y = -1

def callback_position(pose):
	global x, y
	x = pose.x
	y = pose.y
	rospy.loginfo("callback: %f, %f", x, y)
	
	
def controller():

	rospy.init_node('exercise2', anonymous=True)
	
	rospy.wait_for_service("/kill") # By using this function before trying to connect to the server, this program will wait
	# here until the specified service is advertised. Otherwise, if the node providing the service is not ready yet, we'll
	# get an error saying 'unable to connect to service'. So, we should use it always, before calling a service
	client_kill = rospy.ServiceProxy("/kill", Kill)   
	client_kill("turtle1")
	
	rospy.wait_for_service("/spawn")
	client_spawn = rospy.ServiceProxy("/spawn", Spawn)
	client_spawn(5.0, 1.0, 0.0, "rpr_turtle")
	
	rospy.wait_for_service("/rpr_turtle/teleport_absolute")
	client_teleport = rospy.ServiceProxy("/rpr_turtle/teleport_absolute", TeleportAbsolute)
	client_teleport(2.0, 1.0 , 0.0)
	
	rospy.Subscriber("rpr_turtle/pose", Pose, callback_position)
	pub = rospy.Publisher("rpr_turtle/cmd_vel", Twist, queue_size=10)
	
	time.sleep(1) # To allow some time to initialize everything before starting to publish
	
	X_MAX = 9.0
	X_MIN = 2.0
	
	my_vel = Twist() # The vble we'll publish
#	my_vel.linear.x = 1.0
#	pub.publish(my_vel) # Publish an initial command (not necessary in this case)
#	time.sleep(1)
	
	while not rospy.is_shutdown():
		
		rospy.loginfo("main: %f, %f", x, y) # Don't remove this line. Otherwise, it wont't work

		if (x > X_MAX):
			my_vel.linear.x = 1
			my_vel.angular.z = 1
			
		elif (x < X_MIN):
			my_vel.linear.x = 1
			my_vel.angular.z = -1
		
		else:
			my_vel.linear.x = 1
			my_vel.angular.z = 0
			
		pub.publish(my_vel)
			
    
if __name__ == '__main__':
	controller()
	
