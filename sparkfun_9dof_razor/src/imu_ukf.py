#! /usr/bin/env python

import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import State
from sensor_msgs.msg import Imu

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
        self.kalman_state[3,0] = 1.0
        self.is_initialized = True

    @staticmethod
    def prediction(current_state, dt, controls = None):
        pass

    def handle_measurement(self, measurement):
        if not self.is_initialized:
            rospy.logwarn("Filter is unintialized. Discarding measurement")
        else:
            pass

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
