#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# In C++ it was mandatory to have the sequence: init, nodeHandle, advertise.
# However, in python, we do not have nodeHandle, and the init does not need to be the first line,
# it can be after the publisher() function, or before it
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str) # to print the msg into the terminal
        pub.publish(hello_str)
        rate.sleep() # The execution rate of the loop will be 10 Hz

# If we are actually running this script (and not importing it in another script), we execute the
# function talker, which is like the 'main' function
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
