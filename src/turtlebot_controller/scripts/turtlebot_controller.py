#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from my_srv.srv import Velocity, VelocityResponse
from my_srv.msg import Vel

def callback_position(pose):
	rospy.loginfo("The robot is in %f, %f, %f", pose.x, pose.y, pose.theta)
    
def controller(): # main function

	rospy.init_node('controller', anonymous=True)
	rospy.Subscriber("turtle1/pose", Pose, callback_position)
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
	pub2 = rospy.Publisher('my_vel', Vel, queue_size=10)
	
	rospy.wait_for_service("/spawn")
	client = rospy.ServiceProxy("/spawn", Spawn)
    # The name of the service is "/spawn" while the type of the request msg is Spawn    
	resp = client(1.0, 5.0, 0.0, "my_turtle")
	# In resp we get the reply of the serviceServer
	
	rospy.wait_for_service("/velocity")
	client2 = rospy.ServiceProxy("/velocity", Velocity)
    
	rate = rospy.Rate(1) # 1 Hz
	while not rospy.is_shutdown():
		
		new_vel = Vel();
		new_vel.name = "linear"
		new_vel.vel = 0.1
		pub2.publish(new_vel)
				
		resp = client2(0.0, 5.0)
		
		my_vel = Twist()
		my_vel.linear.x = resp.x;
		my_vel.angular.z = resp.z;
		pub.publish(my_vel);
        
        # in python, the spinOnce() function does not exist, only the spin(). Since we are 
        # alreday within a loop, we cannot use spin(), so we just skip it.
        
		rate.sleep() # The execution rate of the loop will be 1 Hz

if __name__ == '__main__':
	controller()
