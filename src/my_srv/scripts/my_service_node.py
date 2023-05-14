#!/usr/bin/env python3
import rospy
import random
from my_srv.srv import Velocity, VelocityResponse
# We need to import both, the whole service type (Velocity) and only the response part (VelocityResponse)

# The callback function takes only the request fields as arguments
def serverCallback(req):
	return VelocityResponse(random.uniform(req.min, req.max),random.uniform(req.min,req.max))
	
def my_vel_server():
	rospy.init_node('my_vel_server')
	s = rospy.Service('velocity', Velocity, serverCallback)
	# 'velocity' is the name of the service while Velocity is the type
	print("Service ready.")
	rospy.spin()


if __name__=="__main__":
	my_vel_server()
