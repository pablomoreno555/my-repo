// The convention is that the names of packages and topics start with lower-case 
// letters, while the names of messages start with capital letters.
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlebot_controller/Vel.h"
#include "my_srv/Velocity.h"

void turtleCallback(const turtlesim::Pose::ConstPtr& msg) {
	ROS_INFO("Turtle subscriber@[%f, %f, %f]", msg->x, msg->y, msg->theta);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 10, turtleCallback);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
	ros::Publisher pub2 = nh.advertise<turtlebot_controller::Vel>("/my_vel", 10);
	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
	// The structure of the serviceClient is similar to the publisher. Both take the initiative
	// (start the communication). However, the publisher does not listen to replies, while the
	// serviceClient waits for the reply of the serviceServer
	
	ros::ServiceClient client2 = nh.serviceClient<my_srv::Velocity>("/velocity");
	
	turtlesim::Spawn spawn_data;
	spawn_data.request.x = 1.0;
	spawn_data.request.y = 1.0;
	spawn_data.request.theta = 0.0;
	spawn_data.request.name = "my_new_turtle";
	
	client.call(spawn_data);
	// Equivalent to pub.publish()
	// We get the reply from the serviceServer in the field spawn.reply.name
	
	ros::Rate rate(1); // The velocity will be sent at a rate of 1s
	
	while (ros::ok())
	{	
		turtlebot_controller::Vel new_vel;
		new_vel.name = "linear";
		new_vel.vel = 1.0;
		pub2.publish(new_vel);
		
		my_srv::Velocity server_vel;
		server_vel.request.min = 0.0;
		server_vel.request.max = 5.0;
		client2.call(server_vel);
		
		geometry_msgs::Twist my_vel;
		my_vel.linear.x = server_vel.response.x;
		my_vel.angular.z = server_vel.response.z;
		pub.publish(my_vel);
		
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

