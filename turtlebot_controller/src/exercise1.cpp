#include "ros/ros.h"
#include <unistd.h>    
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#define X_MAX 10.5
#define Y_MAX 10.5
#define X_MIN 0.5
#define Y_MIN 0.5
#define THR 0.05

double x,y,theta;


void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
ROS_INFO("Turtle subscriber@[%f, %f, %f]",
msg->x, msg->y, msg->theta);
x=msg->x;
y=msg->y;
theta=msg->theta;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1,turtleCallback); 
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	sleep(1);
	geometry_msgs::Twist my_vel;
	my_vel.linear.x = 1.0;
	pub.publish(my_vel);
	sleep(1);
	ros::spinOnce();
	while (ros::ok()){
		if ((x>X_MAX) or (x<X_MIN) or (y>Y_MAX) or (y<Y_MIN)){
			my_vel.linear.x = -my_vel.linear.x;
			pub.publish(my_vel);
			sleep(1);
			}
		else if ((((x-((X_MAX+X_MIN)/2))<THR) and ((y-((Y_MAX+Y_MIN)/2))<THR)) 
		and (((x-((X_MAX+X_MIN)/2))>-THR) and ((y-((Y_MAX+Y_MIN)/2))>-THR))){
			ROS_INFO("%f, %f", x-((X_MAX+X_MIN)/2), (y-((Y_MAX+Y_MIN)/2)));
			my_vel.linear.x = 0;
			my_vel.angular.z = 1.58;
			pub.publish(my_vel);
			sleep(1);
			my_vel.linear.x = 1.0;
			my_vel.angular.z = 0.0;
			pub.publish(my_vel);
			sleep(1);
			}
		else{
			my_vel.linear.x = my_vel.linear.x;
			pub.publish(my_vel);
		}
		ros::spinOnce();
	}
	return 0;
}

