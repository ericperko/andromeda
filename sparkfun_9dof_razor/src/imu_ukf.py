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
        self.gravity_vector = array([0.04,0.,9.8,0.])
        self.magnetic_vector = array([0.03,0.12,0.42,0.])
        self.n = 18
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
        self.initialize_filter()

    def initialize_filter(self, time):
        self.is_initialized = False
        self.time = time
        self.kalman_state = zeros((self.n,1))
        self.kalman_covariance = diag(ones(self.kalman_state.shape[0]))
        self.kalman_state[0:3,0] = \
                tf_math.quaternion_from_euler(0,0,0,'sxyz')[0:3]
        self.is_initialized = True

    @staticmethod
    def rotate_quat_by_omega(quat, omega, dt):
        omega_mag = tf_math.vector_norm(omega)
        omega_unit = tf_math.unit_vector(omega)
        temp_term = 0.5 * omega_mag * dt
        psi_vec = sin(temp_term) * omega_unit
        omega_mat = eye(4) * cos(temp_term)
        omega_mat[0:3,3] = psi_vec
        omega_mat[3,0:3] = -psi_vec
        full_quat = zeros((4,1))
        full_quat[0:3,0] = quat
        full_quat[3,0] = sqrt(1 - dot(quat, quat))
        if not alltrue(isreal(full_quat)):
            #Should find a better exception class for this
            raise Exception("Quat not completely real... this is troubling")
        new_quat = dot(omega_mat, full_quat)
        new_quat = tf_math.unit_vector(new_quat)
        return new_quat[0:3,0]

    @staticmethod
    def prediction(current_state, dt, controls = None):
        #Pass the gyros, accelerations, biases, static magnetic field and
        #gravity straight through
        predicted_state = current_state.copy()
        #Get the best estimate of the current angular rate
        omega = predicted_state[3:6,0]
        #Rotate the previous orientation by this new amount
        new_quat = rotate_quat_by_omega(current_state[0:3,0], omega, dt)
        predicted_state[0:3,0] = new_quat
        #Return a prediction
        return predicted_state

    @staticmethod
    def process_noise(current_state, dt, controls = None):
        return diag(ones(current_state.shape[0]) * 0.1)

    def measurement_update(self, current_state, dt, measurement):
        pass

    def estimate_mean(self, transformed_sigmas):
        est_mean = zeros(self.kalman_state.shape)
        for i in range(0,self.n*2+1):
            est_mean = est_mean + self.weight_mean[i] * transformed_sigmas[i]

    def handle_measurement(self, measurement):
        if not self.is_initialized:
            rospy.logwarn("Filter is unintialized. Discarding measurement")
        else:
            sigmas = self.generate_sigma_points(self.kalman_state,
                    self.kalman_covariance + process_noise())
            dt = (measurement.header.stamp - self.time).to_sec()
            #Run each sigma through the prediction function
            transformed_sigmas = [prediction(sigma, dt) for sigma in sigmas]
            #Estimate the mean
            est_mean = self.estimate_mean(transformed_sigmas)
            est_covariance = zeros(self.kalman_covariance.shape)
            for i in range(0,self.n*2+1):
                diff = transformed_sigmas[i] - est_mean
                prod = dot(diff, diff.T)
                est_covariance = est_covariance + self.weight_covariance[i] * \
                        prod
            est_covariance = est_covariance + self.process_noise(est_mean, dt)
            est_sigmas = self.generate_sigma_points(est_mean, est_covariance)
            self.time = measurement.header.stamp


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
            #Build the noise quaternion
            noise_vec = column[0:3,0]
            noise_quat = ones((4,1))
            noise_mag = tf_math.vector_norm(noise_vec)
            noise_quat_vec_part = noise_vec * sin(noise_mag/2.)/noise_mag
            noise_quat[0:3,0] = noise_quat_vec_part
            noise_quat[3,0] = cos(noise_mag/2.)
            original_quat = mean[0:4,0]
            #Do the additive sample
            perturbed_quat = tf_math.quaternion_multiply(noise_quat,original_quat)
            perturbed_quat = tf_math.unit_vector(perturbed_quat)
            new_mean = mean + column
            new_mean[0:3,0] = perturbed_quat[0:3,0]
            sigmas.append(new_mean)
            #Do the subtractive sample
            conj_noise_quat = tf_math.quaternion_conjugate(noise_quat)
            perturbed_quat = tf_math.quaternion_multiply(conj_noise_quat, original_quat)
            perturbed_quat = tf_math.unit_vector(perturbed_quat)
            new_mean = mean - column
            new_mean[0:3,0] = perturbed_quat[0:3,0]
            sigmas.append(new_mean)
        return sigmas

if __name__ == "__main__":
    rospy.init_node("sf9dof_ukf")
    ukf = SF9DOF_UKF()
    ukf.initialize_filter(rospy.Time.now())
    rospy.Subscriber("state", State, ukf.handle_measurement)
    rospy.spin()
