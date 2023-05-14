#include "ros/ros.h"
#include "my_srv/Velocity.h"
#include <stdlib.h>
#include <stdio.h>

// The arguments of the callback function are both the request fields and the response fields. It will use the request
// fields to do some computation, and modify the response fields. It's always a boolean function: it will return true
// if the request arguments are valid and false otherwise, notifying the serviceClient that the request could not be processed.
// This one will return two random values between req.min and req.max
bool serverCallback(my_srv::Velocity::Request &req, my_srv::Velocity::Response &res)
{
	if (req.min >= req.max) {
		return false;
	}
	else {
		res.x = req.min + (rand()/(RAND_MAX/(req.max-req.min)));
		res.z = req.min + (rand()/(RAND_MAX/(req.max-req.min)));
		return true;
	}
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "velocity_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/velocity", serverCallback);
	// Everytime a "/velocity" service is requested, the "serverCallback" function will be executed, in order to
	// process the service and send the reply, if any.
	// The clientService is equivalent to the publisher (starting the communication), while the serverService is
	// equivalent to the subscriber (processing the request in a callback function)
	
	// This node will be waiting and continuously checking if a service request was made. In that case, it will 
	// process it using the callback function, otherwise, it will keep waiting.
	
	ros::spin();
	return 0;
}
