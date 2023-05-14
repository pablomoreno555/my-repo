#!/usr/bin/env python
import rospy

def main():
	rospy.init_node('a')
	
	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		print("a")
		rate.sleep()


if __name__ == '__main__':
	main()
