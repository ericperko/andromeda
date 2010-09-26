#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest("sparkfun_9dof_razor")
import rospy

from sparkfun_9dof_razor.msg import State
from sparkfun_9dof_razor.msg import RawFilter
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tf_math

from numpy import *
import numpy.linalg

class SF9DOF_UKF:
    def __init__(self):
        self.is_initialized = False
        self.pub = rospy.Publisher("imu_estimate", Imu)
        self.raw_pub = rospy.Publisher("raw_filter", RawFilter)
        self.beta = rospy.get_param("~beta", 2.)
        self.alpha = rospy.get_param("~alpha", 0.001)
        self.kappa = rospy.get_param("~kappa", 0.)
        self.n = 7
        self.kf_lambda = pow(self.alpha,2.) * (self.n + self.kappa) - self.n
        self.weight_covariance = ones(self.n * 2 + 1)
        self.weight_mean = ones(self.n * 2 + 1)
        self.weight_mean = self.weight_mean * (1. / (2 * (self.n +
            self.kf_lambda)))
        self.weight_covariance = self.weight_covariance * (1. / (2 * (self.n +
            self.kf_lambda)))
        self.weight_mean[0] = self.kf_lambda / (self.n + self.kf_lambda)
        self.weight_covariance[0] = self.kf_lambda / (self.n + self.kf_lambda)\
                + (1- pow(self.alpha, 2) + self.beta)

    def initialize_filter(self, time):
        self.time = time
        self.kalman_state = zeros((self.n,1))
        # initalize quaternions
        self.kalman_state[6,0] = 1
        # Guess at magnetic field components
        #self.kalman_state[9,0] = .31
        #self.kalman_state[10,0] = .21
        self.kalman_covariance = diag(ones(self.kalman_state.shape[0]))
        self.is_initialized = True

    @staticmethod
    def prediction(current_state, dt, controls = None):
        predicted_state = current_state.copy()
        x1 = current_state[0,0]
        x2 = current_state[1,0]
        x3 = current_state[2,0]
        x4 = current_state[3,0]
        x5 = current_state[4,0]
        x6 = current_state[5,0]
        x7 = current_state[6,0]
        predicted_state[0,0] = x1
        predicted_state[1,0] = x2
        predicted_state[2,0] = x3
        sqrt2 = 1/(2*(math.sqrt(math.pow(x4,2)+math.pow(x5,2)+math.pow(x6,2)+math.pow(x7,2))))
        predicted_state[3,0] = x4 + sqrt2*(x3*x5-x2*x6+x1*x7)
        predicted_state[4,0] = x5 + sqrt2*(-x3*x4+x1*x6+x2*x7)
        predicted_state[5,0] = x6 + sqrt2*(x2*x4-x1*x5+x3*x7)
        predicted_state[6,0] = x7 + sqrt2*(-x1*x4-x2*x5-x3*x6)
        return predicted_state

    @staticmethod
    def process_noise(current_state, dt, controls = None):
        noise = ones(current_state.shape[0]) * 0.01
        noise[0:3] = 10; # angular velocity uncertanty
        #noise[6:9] = .0001 # gyro bias uncertanty
        #noise[9:11] = .0001 # magnetic field component uncertanty
        #noise[11:14] = 1 # acceleration estimation uncertanty
        return diag(noise)

    @staticmethod
    def measurement_noise(measurement, dt):
        noise = ones(measurement.shape[0]) * 0.01
        noise[0:3] = .01 # magnetomer noise
        noise[3:6] = .01 # gyro noise
        noise[6:9] = .01 # accelerometer noise
        #noise[9:12] = noise[6:9] # fake accelerometer noise
        return diag(noise)

    @staticmethod
    def measurement_update(current_state, dt, measurement):
        predicted_measurement = zeros(measurement.shape)
        x1 = current_state[0,0]
        x2 = current_state[1,0]
        x3 = current_state[2,0]
        a = current_state[3,0]
        b = current_state[4,0]
        c = current_state[5,0]
        d = current_state[6,0]
        R11 = math.pow(d,2)+math.pow(a,2)-math.pow(b,2)-math.pow(c,2)
        R12 = 2*(a*b-c*d)
        R13 = 2*(a*c+b*d)
        R21 = 2*(a*b+c*d)
        R22 = math.pow(d,2)+math.pow(b,2)-math.pow(a,2)-math.pow(c,2)
        R23 = 2*(b*c-a*d)
        R31 = 2*(a*c-b*d)
        R32 = 2*(b*c+a*d)
        R33 = math.pow(d,2)+math.pow(c,2)-math.pow(b,2)-math.pow(a,2)
        denom = math.pow(a,2)+math.pow(b,2)+math.pow(c,2)+math.pow(d,2)
        g1 = 0
        g2 = 0
        g3 = -9.81
        h1 = .28
        h2 = 0
        h3 = .22
        #Calculate the predicted magnetometer reading
        predicted_measurement[0,0] = (R11*h1+R12*h2+R13*h3)/denom
        predicted_measurement[1,0] = (R21*h1+R22*h2+R23*h3)/denom
        predicted_measurement[2,0] = (R31*h1+R32*h2+R33*h3)/denom
        #Calculate the predicted gyro readings
        predicted_measurement[3,0] = x1
        predicted_measurement[4,0] = x2
        predicted_measurement[5,0] = x3
        #Calculate the predicted accelerometer readings
        predicted_measurement[6,0] = (R11*g1+R12*g2+R13*g3)/denom
        predicted_measurement[7,0] = (R21*g1+R22*g2+R23*g3)/denom
        predicted_measurement[8,0] = (R31*g1+R32*g2+R33*g3)/denom
        return predicted_measurement

    def estimate_mean(self, transformed_sigmas):
        est_mean = zeros(self.kalman_state.shape)
        #Compute estimated mean for non-quaternion components
        for i in range(0,self.n*2+1):
            est_mean += self.weight_mean[i] * transformed_sigmas[i]
        return est_mean

    def estimate_covariance(self, est_mean, transformed_sigmas):
        est_covariance = zeros(self.kalman_covariance.shape)
        diff = zeros(est_mean.shape)
        for i in range(0,self.n*2+1):
            diff = transformed_sigmas[i] - est_mean
            prod = dot(diff, diff.T)
            term = self.weight_covariance[i] * prod
            est_covariance += term
        return est_covariance

    def estimate_measurement_mean(self, measurement_sigmas):
        est_measurement = zeros(measurement_sigmas[0].shape)
        for i in range(0, self.n*2+1):
            term = self.weight_mean[i] * measurement_sigmas[i]
            est_measurement += term
        return est_measurement

    def estimate_measurement_covariance(self, measurement_mean, \
            measurement_sigmas):
        est_measurement_covariance = eye(measurement_mean.shape[0])
        for i in range(0,self.n*2+1):
            diff = measurement_sigmas[i] - measurement_mean
            prod = dot(diff, diff.T)
            term = self.weight_covariance[i] * prod
            est_measurement_covariance += term
        return est_measurement_covariance

    def cross_correlation_mat(self, est_mean, est_sigmas, meas_mean, meas_sigmas):
        cross_correlation_mat = zeros((est_mean.shape[0], meas_mean.shape[0]))
        est_diff = zeros(est_mean.shape)
        for i in range(0,self.n*2+1):
            est_diff = est_sigmas[i] - est_mean
            meas_diff = meas_sigmas[i] - meas_mean
            prod = dot(est_diff, meas_diff.T)
            term = self.weight_covariance[i] * prod
            cross_correlation_mat += term
        return cross_correlation_mat

    @staticmethod
    def stateMsgToMat(measurement_msg):
        measurement = zeros((9,1))
        measurement[0,0] = measurement_msg.magnetometer.x
        measurement[1,0] = measurement_msg.magnetometer.y
        measurement[2,0] = measurement_msg.magnetometer.z
        measurement[3,0] = measurement_msg.angular_velocity.x
        measurement[4,0] = measurement_msg.angular_velocity.y
        measurement[5,0] = measurement_msg.angular_velocity.z
        measurement[6,0] = measurement_msg.linear_acceleration.x
        measurement[7,0] = measurement_msg.linear_acceleration.y
        measurement[8,0] = -measurement_msg.linear_acceleration.z
        return measurement

    def handle_measurement(self, measurement_msg):
        if not self.is_initialized:
            rospy.logwarn("Filter is unintialized. Discarding measurement")
        else:
            t0 = rospy.Time.now()
            dt = (measurement_msg.header.stamp - self.time).to_sec()
            measurement = SF9DOF_UKF.stateMsgToMat(measurement_msg)
            p_noise = SF9DOF_UKF.process_noise(self.kalman_state, dt)
            sigmas = self.generate_sigma_points(self.kalman_state,
                    self.kalman_covariance + p_noise)
            #Run each sigma through the prediction function
            transformed_sigmas = [SF9DOF_UKF.prediction(sigma, dt) for sigma in sigmas]
            #Estimate the mean
            est_mean = self.estimate_mean(transformed_sigmas)
            est_covariance = self.estimate_covariance(est_mean, \
                    transformed_sigmas)
            est_sigmas = self.generate_sigma_points(est_mean, est_covariance)
            #Run each of the new sigmas through the measurement update function
            measurement_sigmas = [SF9DOF_UKF.measurement_update(sigma, dt, \
                    measurement) for sigma in est_sigmas]
            measurement_mean = \
                    self.estimate_measurement_mean(measurement_sigmas)
            measurement_covariance = \
                    self.estimate_measurement_covariance(measurement_mean, \
                    measurement_sigmas)
            measurement_covariance += SF9DOF_UKF.measurement_noise(measurement_mean, dt)
            cross_correlation_mat = self.cross_correlation_mat(est_mean, \
                    est_sigmas, measurement_mean, measurement_sigmas)
            s_inv = numpy.linalg.pinv(measurement_covariance)
            kalman_gain = dot(cross_correlation_mat, s_inv)
            innovation =  measurement - measurement_mean
            innovation = innovation
            correction = dot(kalman_gain, innovation)
            self.kalman_state = est_mean + correction
            # Sneak in raw accelerometers
            #self.kalman_state[11:14] = measurement[6:9]
            temp = dot(kalman_gain, measurement_covariance)
            temp = dot(temp, kalman_gain.T)
            self.kalman_covariance = est_covariance - temp
            self.time = measurement_msg.header.stamp
            self.publish_imu()
            self.publish_raw_filter()
            rospy.logdebug("Took %s sec to run filter",(rospy.Time.now() - \
                    t0).to_sec())
        
    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.time
        imu_msg.header.frame_id = 'imu_odom'
        #quat = tf_math.quaternion_from_euler(self.kalman_state[0,0],self.kalman_state[1,0],self.kalman_state[2,0], axes='sxyz')
        a = self.kalman_state[3,0]
        b = self.kalman_state[4,0]
        c = self.kalman_state[5,0]
        d = self.kalman_state[6,0]
        q = math.sqrt(math.pow(a,2)+math.pow(b,2)+math.pow(c,2)+math.pow(d,2))
        angles = tf_math.euler_from_quaternion(self.kalman_state[3:7,0])
        imu_msg.orientation.x = angles[0]#a/q
        imu_msg.orientation.y = angles[1]#b/q
        imu_msg.orientation.z = angles[2]#c/q
        imu_msg.orientation.w = 0#d/q
        imu_msg.orientation_covariance = list(self.kalman_covariance[3:6,3:6].flatten())
        imu_msg.angular_velocity.x = self.kalman_state[0,0]
        imu_msg.angular_velocity.y = self.kalman_state[1,0]
        imu_msg.angular_velocity.z = self.kalman_state[2,0]
        imu_msg.angular_velocity_covariance = list(self.kalman_covariance[0:3,0:3].flatten())
        imu_msg.linear_acceleration.x = 0#self.kalman_state[11,0]
        imu_msg.linear_acceleration.y = 0#self.kalman_state[12,0]
        imu_msg.linear_acceleration.z = 0#self.kalman_state[13,0]
        imu_msg.linear_acceleration_covariance = [.01, 0, 0, 0, .01, 0, 0, 0, .01]#list(self.kalman_covariance[11:14,11:14].flatten())
        self.pub.publish(imu_msg)

    def publish_raw_filter(self):
        filter_msg = RawFilter()
        filter_msg.header.stamp = self.time
        filter_msg.state = list(self.kalman_state.flatten())
        filter_msg.covariance = list(self.kalman_covariance.flatten())
        self.raw_pub.publish(filter_msg)

    def generate_sigma_points(self, mean, covariance):
        sigmas = []
        sigmas.append(mean)
        temp = numpy.linalg.cholesky(covariance)
        temp = temp * sqrt(self.n + self.kf_lambda)
        for i in range(0,self.n):
            #Must use columns in order to get the write thing out of the
            #Cholesky decomposition
            #(http://en.wikipedia.org/wiki/Cholesky_decomposition#Kalman_filters)
            column = temp[:,i].reshape((self.n,1))
            #Do the additive sample
            new_mean = mean + column
            sigmas.append(new_mean)
            #Do the subtractive sample
            new_mean = mean - column
            sigmas.append(new_mean)
        return sigmas

if __name__ == "__main__":
    rospy.init_node("sf9dof_ukf_quat", log_level=rospy.DEBUG)
    ukf = SF9DOF_UKF()
    ukf.initialize_filter(rospy.Time.now())
    rospy.Subscriber("state", State, ukf.handle_measurement)
    rospy.spin()
