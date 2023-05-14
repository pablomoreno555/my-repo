#!/usr/bin/env python
import rospy
from turtlesim.srv import Spawn, SpawnResponse

def main():
	rospy.init_node('node_q7', anonymous=True)
	rospy.wait_for_service("/spawn")
	client = rospy.ServiceProxy("/spawn", Spawn)
	resp = client(1.0, 1.0, 0.0, "my_turtle")
	print(resp.name)

if __name__ == '__main__':
	main()
