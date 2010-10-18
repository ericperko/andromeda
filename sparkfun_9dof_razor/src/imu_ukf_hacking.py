#! /usr/bin/env python

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
import scipy.linalg.decomp
import numpy.testing
import pdb

def pos_sem_def_sqrt(mat):
    eigvals, eigvecs = scipy.linalg.decomp.eigh(mat)
    rospy.loginfo("eigvals: %s", eigvals)
    diag_sqrt = diag(sqrt(eigvals))
    sqrt_mat = dot(dot(eigvecs, diag_sqrt), eigvecs.T)
    return sqrt_mat


def check_symmetry(mat):
        try:
            numpy.testing.assert_almost_equal(mat, mat.T)
            return True
        except AssertionError as e:
            rospy.logerr(e)
            return False

class SF9DOF_UKF:
    def __init__(self):
        self.pub = rospy.Publisher("imu_estimate", Imu)
        self.raw_pub = rospy.Publisher("raw_filter", RawFilter)
        self.is_initialized = False
        self.beta = rospy.get_param("~beta", 2.)
        self.alpha = rospy.get_param("~alpha", 0.001)
        self.kappa = rospy.get_param("~kappa", 0.)
        self.gravity_vector = array([0.04,0.,9.8])
        self.magnetic_vector = array([0.03,0.12,0.42])
        self.n = 6
        self.kf_lambda = pow(self.alpha,2.) * (self.n + self.kappa) - self.n
        self.weight_covariance = ones(self.n * 2 + 1)
        self.weight_mean = ones(self.n * 2 + 1)
        self.weight_mean = self.weight_mean * (1. / (2 * (self.n + \
                self.kf_lambda)))
        self.weight_covariance = self.weight_covariance * (1. / (2 * (self.n +\
                self.kf_lambda)))
        self.weight_mean[0] = self.kf_lambda / (self.n + self.kf_lambda)
        self.weight_covariance[0] = self.kf_lambda / (self.n + self.kf_lambda)\
                + (1- pow(self.alpha, 2) + self.beta)

    def initialize_filter(self, time):
        self.is_initialized = False
        self.time = time
        self.kalman_state = zeros((self.n,1))
        self.kalman_covariance = diag(ones(self.kalman_state.shape[0]) * 1.)
        check_symmetry(self.kalman_covariance)
        self.kalman_state[0:3,0] = \
                tf_math.quaternion_from_euler(0,0,0,'sxyz')[0:3]
        self.is_initialized = True

    @staticmethod
    def rotate_quat_by_omega(quat, omega, dt):
        omega_mag = tf_math.vector_norm(omega)
        temp_term = 0.5 * omega_mag * dt
        psi_vec = sin(temp_term) * omega
        if omega_mag != 0.:
            psi_vec = psi_vec / omega_mag
        omega_mat = eye(4) * cos(temp_term)
        omega_mat[0:3,3] = psi_vec
        omega_mat[3,0:3] = -psi_vec
        full_quat = zeros((4,1))
        full_quat[0:3,0] = quat
        full_quat[3,0] = sqrt(1 - dot(quat, quat))
        if not alltrue(isreal(full_quat)):
            #TODO Should find a better exception class for this
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
        new_quat = SF9DOF_UKF.rotate_quat_by_omega(current_state[0:3,0], omega, dt)
        predicted_state[0:3,0] = new_quat
        #Return a prediction
        return predicted_state

    @staticmethod
    def recover_quat(mean):
        quat_temp = zeros((4,1))
        quat_vec = mean[0:3,0]
        quat_temp[0:3,0] = quat_vec
        quat_temp[3,0] = 1. - tf_math.vector_norm(quat_vec)
        return quat_temp

    @staticmethod
    def build_noise_quat(noise_vec):
        noise_quat = ones((4,1))
        noise_mag = tf_math.vector_norm(noise_vec)
        if noise_mag != 0.:
            noise_quat_vec_part = noise_vec * sin(noise_mag/2.)/noise_mag
        else:
            noise_quat_vec_part = noise_vec * sin(noise_mag/2.)
        noise_quat[0:3,0] = noise_quat_vec_part
        noise_quat[3,0] = cos(noise_mag/2.)
        return noise_quat

    @staticmethod
    def correct_mean(est_mean, correction):
        error_quat = SF9DOF_UKF.recover_quat(correction)
        original_quat = SF9DOF_UKF.recover_quat(est_mean)
        corrected_quat = tf_math.quaternion_multiply(error_quat, original_quat)
        corrected_mean = est_mean + correction
        corrected_mean[0:3,0] = corrected_quat[0:3,0]
        return corrected_mean

    @staticmethod
    def process_noise(current_state, dt, controls = None):
        noise = ones(current_state.shape[0]) * 1e-3
        return diag(noise)

    @staticmethod
    def measurement_noise(measurement, dt):
        return diag(ones(measurement.shape[0]) * 1e-2)

    @staticmethod
    def measurement_update(current_state, dt, measurement):
        predicted_measurement = zeros(measurement.shape)
        #orientation = SF9DOF_UKF.recover_quat(current_state)
        #Calculate predicted magnetometer readings
        #h_vec = zeros((4,1))
        #h_vec[0:3,0] = current_state[12:15,0]
        #h_vec[3,0] = 0.
        #temp = tf_math.quaternion_multiply(orientation, h_vec)
        #result = tf_math.quaternion_multiply(temp, \
        #        tf_math.quaternion_conjugate(orientation))
        #predicted_measurement[0:3,0] = result[0:3,0]
        #Calculate the predicted gyro readings
        temp_gyro = current_state[3:6,0]
        predicted_measurement[0:3,0] = temp_gyro
        #Calculate the predicted accelerometer readings
        #g_vec = zeros((4,1))
        #g_vec[0:3,0] = current_state[15:18,0]
        #g_vec[3,0] = 0.
        #temp = tf_math.quaternion_multiply(orientation, g_vec)
        #result = tf_math.quaternion_multiply(temp, \
        #        tf_math.quaternion_conjugate(orientation))
        #temp_accel = current_state[6:9,0] + result[0:3,0]
        #predicted_measurement[6:9,0] = temp_accel
        return predicted_measurement

    def estimate_mean(self, transformed_sigmas):
        quat_sum = zeros((4,1))
        est_mean = zeros(self.kalman_state.shape)
        #Compute estimated mean for non-quaternion components
        for i in range(0,self.n*2+1):
            est_mean += self.weight_mean[i] * transformed_sigmas[i]
        #Compute quaternion component mean
        for i in range(0, self.n*2+1):
            quat_temp = SF9DOF_UKF.recover_quat(transformed_sigmas[i])
            quat_sum += self.weight_mean[i] * quat_temp
        #This should create a unit quaternion
        est_quat = quat_sum / tf_math.vector_norm(quat_sum)
        est_mean[0:3,0] = est_quat[0:3,0]
        return est_mean

    def estimate_covariance(self, est_mean, transformed_sigmas):
        est_covariance = zeros(self.kalman_covariance.shape)
        diff = zeros(est_mean.shape)
        #Calculate the inverse mean quat
        mean_quat = SF9DOF_UKF.recover_quat(est_mean)
        inverse_mean_quat = tf_math.quaternion_inverse(mean_quat.flatten())
        for i in range(0,self.n*2+1):
            #Calculate the error for the quaternion
            sig_quat = SF9DOF_UKF.recover_quat(transformed_sigmas[i])
            error_quat = tf_math.quaternion_multiply(sig_quat, \
                    inverse_mean_quat)
            diff[0:3,0] = error_quat[0:3,0]
            #Calculate the error for the rest of the state
            diff[3:,0] = transformed_sigmas[i][3:,0] - est_mean[3:,0]
            prod = dot(diff, diff.T)
            term = self.weight_covariance[i] * prod
            est_covariance += term
        check_symmetry(est_covariance)
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
        check_symmetry(est_measurement_covariance)
        return est_measurement_covariance

    def cross_correlation_mat(self, est_mean, est_sigmas, meas_mean, meas_sigmas):
        cross_correlation_mat = zeros((est_mean.shape[0], meas_mean.shape[0]))
        est_diff = zeros(est_mean.shape)
        mean_quat = SF9DOF_UKF.recover_quat(est_mean)
        inverse_mean_quat = tf_math.quaternion_inverse(mean_quat.flatten())
        for i in range(0,self.n*2+1):
            #Calculate the error for the quaternion
            sig_quat = SF9DOF_UKF.recover_quat(est_sigmas[i])
            error_quat = tf_math.quaternion_multiply(sig_quat, \
                    inverse_mean_quat)
            est_diff[0:3,0] = error_quat[0:3,0]
            #Calculate the error for the rest of the state
            est_diff[3:,0] = est_sigmas[i][3:,0] - est_mean[3:,0]
            meas_diff = meas_sigmas[i] - meas_mean
            prod = dot(est_diff, meas_diff.T)
            term = self.weight_covariance[i] * prod
            cross_correlation_mat += term
        return cross_correlation_mat

    @staticmethod
    def stateMsgToMat(measurement_msg):
        measurement = zeros((3,1))
        #measurement[0,0] = measurement_msg.magnetometer.x
        #measurement[1,0] = measurement_msg.magnetometer.y
        #measurement[2,0] = measurement_msg.magnetometer.z
        measurement[0,0] = measurement_msg.angular_velocity.x
        measurement[1,0] = measurement_msg.angular_velocity.y
        measurement[2,0] = measurement_msg.angular_velocity.z
        #measurement[6,0] = measurement_msg.linear_acceleration.x
        #measurement[7,0] = measurement_msg.linear_acceleration.y
        #measurement[8,0] = measurement_msg.linear_acceleration.z
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
            correction = dot(kalman_gain, innovation)
            self.kalman_state = SF9DOF_UKF.correct_mean(est_mean, correction)
            temp = dot(kalman_gain, measurement_covariance)
            temp = dot(temp, kalman_gain.T)
            self.kalman_covariance = est_covariance - temp
            check_symmetry(self.kalman_covariance)
            self.time = measurement_msg.header.stamp
            self.publish_imu()
            self.publish_raw_filter()
            rospy.logdebug("Took %s sec to run filter",(rospy.Time.now() - \
                t0).to_sec())

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.time
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation = Quaternion(*SF9DOF_UKF.recover_quat(self.kalman_state))
        imu_msg.orientation_covariance = \
               list(self.kalman_covariance[0:3,0:3].flatten())
        imu_msg.angular_velocity.x = self.kalman_state[3,0]
        imu_msg.angular_velocity.y = self.kalman_state[4,0]
        imu_msg.angular_velocity.z = self.kalman_state[5,0]
        imu_msg.angular_velocity_covariance = \
                list(self.kalman_covariance[3:6,3:6].flatten())
        #imu_msg.linear_acceleration.x = self.kalman_state[6,0]
        #imu_msg.linear_acceleration.y = self.kalman_state[7,0]
        #imu_msg.linear_acceleration.z = self.kalman_state[8,0]
        #imu_msg.linear_acceleration_covariance = \
        #       list(self.kalman_covariance[6:9,6:9].flatten())
        self.pub.publish(imu_msg)
        rospy.loginfo("Orientation was %s",\
                tf_math.euler_from_quaternion(SF9DOF_UKF.recover_quat(self.kalman_state).flatten()))

    def publish_raw_filter(self):
        filter_msg = RawFilter()
        filter_msg.header.stamp = self.time
        filter_msg.state = list(self.kalman_state.flatten())
        filter_msg.covariance = list(self.kalman_covariance.flatten())
        self.raw_pub.publish(filter_msg)

    def generate_sigma_points(self, mean, covariance):
        sigmas = []
        sigmas.append(mean)
        if not check_symmetry(covariance):
            pdb.set_trace()
        temp = pos_sem_def_sqrt(covariance)
        if any(isnan(temp)):
            rospy.logerr("Sqrt matrix contained a NaN. Matrix was %s",
                    covariance)
        temp = temp * sqrt(self.n + self.kf_lambda)
        for i in range(0,self.n):
            #Must use columns in order to get the write thing out of the
            #Cholesky decomposition
            #(http://en.wikipedia.org/wiki/Cholesky_decomposition#Kalman_filters)
            column = temp[:,i].reshape((self.n,1))
            #Build the noise quaternion
            noise_vec = column[0:3,0]
            noise_quat = SF9DOF_UKF.build_noise_quat(noise_vec)
            original_quat = SF9DOF_UKF.recover_quat(mean)
            #Do the additive sample
            perturbed_quat = tf_math.quaternion_multiply(noise_quat,original_quat)
            #perturbed_quat = tf_math.unit_vector(perturbed_quat)
            new_mean = mean + column
            new_mean[0:3,0] = perturbed_quat[0:3,0]
            sigmas.append(new_mean)
            #Do the subtractive sample
            conj_noise_quat = tf_math.quaternion_conjugate(noise_quat)
            perturbed_quat = tf_math.quaternion_multiply(conj_noise_quat, original_quat)
            #perturbed_quat = tf_math.unit_vector(perturbed_quat)
            new_mean = mean - column
            new_mean[0:3,0] = perturbed_quat[0:3,0]
            sigmas.append(new_mean)
        return sigmas

if __name__ == "__main__":
    rospy.init_node("sf9dof_ukf", log_level=rospy.DEBUG)
    ukf = SF9DOF_UKF()
    ukf.initialize_filter(rospy.Time.now())
    rospy.Subscriber("state", State, ukf.handle_measurement)
    rospy.spin()
