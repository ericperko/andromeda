#! /usr/bin/env python
import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import State

def handle_imu_state(msg, f):
	data_string = "%f, %f, %f, %f, %f, %f, %f, %f, %f\n" % (msg.magnetometer.x, msg.magnetometer.y, msg.magnetometer.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
	f.write(data_string)


if __name__ == "__main__":
	rospy.init_node('imu_to_csv')
	with open(rospy.get_param("~filename"), "w") as file:
	    rospy.Subscriber('state', State, handle_imu_state, file)
	    rospy.spin()
