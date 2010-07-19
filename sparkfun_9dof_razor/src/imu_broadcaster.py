#! /usr/bin/env python

import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import State

import serial

class SF9DOF_Broadcaster:
    def __init__(self, port, magneto_err, gyro_err, accel_err):
        self.port = port
        self.mag_err = magneto_err
        self.gyro_err = gyro_err
        self.accel_err = accel_err
        self.pub = rospy.Publisher("state", State)
        self.message = State()
        self.message.header.frame_id = "sf9dof_link"
        self.message.mag_error = self.mag_err
        self.message.omega_error = self.gyro_err
        self.message.accel_error = self.accel_err

    def loop(self):
        while not rospy.is_shutdown():
            line = port.readline()
            chunks = line.split(":")
            if chunks[0] == "!IMU":
                readings = chunks[1].split(",")
                self.message.header.stamp = rospy.Time.now()
                self.message.magnetometer.x = float(readings[0])
                self.message.magnetometer.y = float(readings[1])
                self.message.magnetometer.z = float(readings[2])
                self.message.angular_velocity.x = float(readings[3])
                self.message.angular_velocity.y = float(readings[4])
                self.message.angular_velocity.z = float(readings[5])
                self.message.linear_acceleration.x = float(readings[6])
                self.message.linear_acceleration.y = float(readings[7])
                self.message.linear_acceleration.z = float(readings[8])
                self.pub.publish(self.message)
            else:
                rospy.logerr("Did not get a valid IMU packet, got %s", line)

if __name__ == "__main__":
    rospy.init_node('sparkfun_9dof_razor')
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
    port = serial.Serial(port_name, 115200, timeout = 1)
    magneto_error = rospy.get_param("~magneto_error", 0.1)
    gyro_error = rospy.get_param("~gyro_error", 0.1)
    accel_error = rospy.get_param("~accel_error", 0.1)
    broadcaster = SF9DOF_Broadcaster(port, magneto_error, gyro_error, accel_error)
    broadcaster.loop()
    port.close()
