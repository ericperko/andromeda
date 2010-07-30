#! /usr/bin/env python

import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import State
from sensor_msgs.msg import Imu
import tf.transformations as tf_math

from numpy import *
import numpy.linalg

class SF9DOF_UKF:
    def __init__(self):
        self.pub = rospy.Publisher("imu_estimate", Imu)
        self.is_initialized = False
        self.beta = rospy.get_param("~beta", 2)
        self.alpha = rospy.get_param("~alpha", 2)
        self.kappa = rospy.get_param("~kappa", 1.5)
        self.n = 10
        self.kf_lambda = pow(self.alpha,2.) * (self.n + self.kappa) - self.n
        self.weight_covariance = ones(self.n * 2 + 1)
        self.weight_mean = ones(self.n * 2 + 1)
        self.weight_mean = self.weight_mean * (1. / (2 * (self.n +
            self.kf_lambda)))
        self.weight_covariance = self.weight_covariance * (1. / (2 * (self.n +
            self.kf_lambda)))
        self.weight_mean[0] = self.kf_lambda / (self.n + self.kf_lambda)
        self.weight_covariance[0] = self.kf_lambda / (self.n + self.kf_lambda) + (1 -
                pow(self.alpha, 2) + self.beta)
        self.initialize_filter()

    def initialize_filter(self):
        self.is_initialized = False
        self.time = rospy.Time.now()
        self.kalman_state = zeros((self.n,1))
        self.kalman_covariance = diag(ones(self.kalman_state.shape[0]))
        self.kalman_state[0:4,0] = tf_math.quaternion_from_euler(0,0,0,'sxyz')
        self.is_initialized = True

    @staticmethod
    def prediction(current_state, dt, controls = None):
        #Pass the gyro and accelerations straight through
        predicted_state = current_state.copy()
        #Generate the quaternion for the rotation given by the gyros
        angles = current_state[4:7,0] * dt
        quat = tf_math.quaternion_from_euler(*angles, axes='sxyz')
        #Rotate the previous orientation by this new amount
        new_angle = tf_math.quaternion_multiply(quat, current_state[0:4,0])
        predicted_state[0:4,0] = new_angle
        #Return a prediction
        return predicted_state

    def handle_measurement(self, measurement):
        if not self.is_initialized:
            rospy.logwarn("Filter is unintialized. Discarding measurement")
        else:
            sigmas = generate_sigma_points(self.kalman_state,
                    self.kalman_covariance)
            dt = (measurement.header.stamp - self.time).to_sec()
            transformed_sigmas = [prediction(sigma, dt) for sigma in sigmas]


    def generate_sigma_points(self, mean, covariance):
        sigmas = []
        sigmas.append(mean)
        temp = (self.n + self.kf_lambda) * covariance
        temp = numpy.linalg.cholesky(temp)
        for i in range(0,self.n):
            #Must use columns in order to get the write thing out of the
            #Cholesky decomposition
            #(http://en.wikipedia.org/wiki/Cholesky_decomposition#Kalman_filters)
            sigmas.append(mean + temp[:,i].reshape(10,1))
            sigmas.append(mean - temp[:,i].reshape(10,1))
        return sigmas

if __name__ == "__main__":
    rospy.init_node("sf9dof_ukf")
    ukf = SF9DOF_UKF()
    rospy.Subscriber("state", State, ukf.handle_measurement)
    rospy.spin()
