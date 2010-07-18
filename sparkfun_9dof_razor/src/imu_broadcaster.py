#! /usr/bin/env python

import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import 9DOF_State

import serial

class SF9DOF_Broadcaster:
	def __init__(self, port):
		pass

if __name__ == "__main__":
	rospy.init_node('sparkfun_9dof_razor')
	broadcaster = SF9DOF_Broadcaster()
	rospy.spin()
