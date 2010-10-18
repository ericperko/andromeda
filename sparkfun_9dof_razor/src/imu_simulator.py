#! /usr/bin/env python

import roslib
roslib.load_manifest('sparkfun_9dof_razor')
import rospy

from sparkfun_9dof_razor.msg import State
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_math

from numpy import *
import numpy.linalg

import imu_ukf

if __name__ == "__main__":
    rospy.init_node("imu_sim")
    omega = zeros((3,1))
    state_pub = rospy.Publisher('state_sim', State)
    ground_truth_pub = rospy.Publisher('imu_ground_truth', Imu)
    gravity_vector = array([0.04,0.,9.8])
    magnetic_vector = array([0.03,0.12,0.42])
    yaw = 0.0
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        yaw += 0.01
        i = Imu()
        i.header.stamp = rospy.Time.now()
        i.orientation =\
                Quaternion(*tf_math.quaternion_from_euler(0,0,yaw,'sxyz'))
        i.angular_velocity.z = 0.01

        s = State()
        s.header.stamp = rospy.Time.now()
        s.magnetometer.x = magnetic_vector[0]
        s.magnetometer.y = magnetic_vector[1]
        s.magnetometer.z = magnetic_vector[2]
        s.linear_acceleration.x = gravity_vector[0]
        s.linear_acceleration.y = gravity_vector[1]
        s.linear_acceleration.z = gravity_vector[2]
        s.angular_velocity.z = 0.01

        state_pub.publish(s)
        ground_truth_pub.publish(i)
        r.sleep()
