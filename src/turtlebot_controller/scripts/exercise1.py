#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time

# Global vbles
x = -1
y = -1

def callback_controller(pose):
	global x, y # To indicate that we're referening to the global vbles. Otherwise, two local vbles would be created, with the
	# same names (x,y), but different from the global ones
	x = pose.x
	y = pose.y
	rospy.loginfo("callback: %f, %f", x, y)


def controller():

	rospy.init_node('exercise1', anonymous=True)
	rospy.Subscriber("turtle1/pose", Pose, callback_controller)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
	
	time.sleep(1) # To allow some time to initialize everything before starting to publish
	
	X_MAX = 10.5
	Y_MAX = 10.5
	X_MIN = 0.5
	Y_MIN = 0.5
	X0 = (X_MAX + X_MIN) / 2
	Y0 = (Y_MAX + Y_MIN) / 2
	THR = 0.05 # threshold
	
	my_vel = Twist() # The vble we'll publish
	my_vel.linear.x = 1.0
	pub.publish(my_vel)
	time.sleep(1)
	
	while not rospy.is_shutdown():
		
		rospy.loginfo("main: %f, %f", x, y) # Don't remove this line. Otherwise, it wont't work

		if (x > X_MAX or x < X_MIN or y > Y_MAX or y < Y_MIN):
			my_vel.linear.x = -my_vel.linear.x
			pub.publish(my_vel)
			time.sleep(1)
		
		elif ((x-X0) < THR and (y-Y0) < THR  and (x-X0) > -THR and (y-Y0) > -THR): # hysteresis
			my_vel.linear.x = 0.0
			my_vel.angular.z = 1.58
			pub.publish(my_vel)
			time.sleep(1)
			
			my_vel.linear.x = 1.0
			my_vel.angular.z = 0.0
			pub.publish(my_vel)
			time.sleep(1)
		
		my_vel.linear.x = my_vel.linear.x
		my_vel.angular.z = 0
		pub.publish(my_vel)
			
    
if __name__ == '__main__':
	controller()
	
