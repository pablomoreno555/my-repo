#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

// In the subscribe() function we don't specify the smsg type. We specify it in the
// argument of the callback function. The msg received is actually a pointer to a
// std_msgs::String
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str()); // We print the msg received
  // The arrow is because msg is a pointer -> to access the field data we need to use the
  // dereferencing operator (->). The c_str() function is to convert to string
}
// We should always try to do the callback function as short as possible, since we want
// the subscriber to be able to not miss any msg that the publishes sends

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // Every time a msg is received by the chatter topic, the function chatterCallback
  // will be executed. Every subscriber is associated to a callback function
 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one). ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  // It is continuosly checking if a new msg is received. It is equivalent to the
  // spinOnce() function, but within a while loop: while(1) {spinOnce()}.

  return 0;
}
