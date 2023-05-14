#!/usr/bin/env python3
import rospy
import math
from my_srv2.srv import Pos2Vel, Pos2VelResponse
# We need to import both, the whole service type and only the response part

# The callback function takes only the request fields as arguments
def serverCallback(req):
	return Pos2VelResponse(0.1 + 2*math.sin(math.pi * req.pos/7 - 2 * math.pi/7))
	
def harmonic_server():
	rospy.init_node('harmonic_server')
	s = rospy.Service('harmonic_service', Pos2Vel, serverCallback)
	# 'harmonic_service' is the name of the service while Pos2Vel is the type (request/reply structure)
	print("Service ready!")
	rospy.spin()


if __name__=="__main__":
	harmonic_server()
