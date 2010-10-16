#! /usr/bin/env python

# Attempt # 1

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
        self.n = 11
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
        # Guess at magnetic field components
        self.kalman_state[9,0] = .31
        self.kalman_state[10,0] = .21
        self.kalman_covariance = diag(ones(self.kalman_state.shape[0]))
        self.is_initialized = True

    @staticmethod
    def limitAngles(limitState):
        # Don't care about poll/pitch, would gimbal lock anyway
        roll = limitState[0]
        pitch = limitState[1]
        # Limit heading to between 0 and 2 pi
        heading = limitState[2]
        if(heading < 0):
            limitState[2] = heading + 2*math.pi
        elif(heading > 2*math.pi):
            limitState[2] = heading - 2*math.pi
        return limitState

    @staticmethod
    def limitTheta(theta):
        # Limit heading to between 0 and 2 pi
        theta = theta % (2*math.pi)
        if(theta< 0):
            theta = theta+2*math.pi
        return theta

    @staticmethod
    def limitInnovationAngles(innovation):
        # Limit heading innovation to between + pi and - pi
        dheading = innovation[2]
        if(dheading < -math.pi):
            innovation[2] = dheading + 2*math.pi
        elif(dheading > math.pi):
            innovation[2] = dheading - 2*math.pi
        return innovation

    @staticmethod
    def prediction(current_state, dt, controls = None):
        predicted_state = current_state.copy()
        omegas = current_state[3:6]
        predicted_state[0:3] = current_state[0:3] + omegas*dt
        predicted_state = predicted_state
        return predicted_state

    @staticmethod
    def process_noise(current_state, dt, controls = None):
        noise = ones(current_state.shape[0]) * 0.01
        noise[3:6] = 10; # angular velocity uncertanty
        noise[6:9] = .0001 # gyro bias uncertanty
        noise[9:11] = .0001 # magnetic field component uncertanty
        return diag(noise)

    @staticmethod
    def measurement_noise(measurement, dt):
        noise = ones(measurement.shape[0]) * 0.01
        noise[0:3] = .01 # magnetomer noise
        noise[3:6] = .01 # gyro noise
        noise[6:9] = .01 # accelerometer noise
        return diag(noise)

    @staticmethod
    def measurement_update(current_state, dt, measurement):
        predicted_measurement = zeros(measurement.shape)
        r = current_state[0,0]
        p = current_state[1,0]
        w = current_state[2,0]
        vr = current_state[3,0]
        vp = current_state[4,0]
        vw = current_state[5,0]
        br = current_state[6,0]
        bp = current_state[7,0]
        bw = current_state[8,0]
        mxy = current_state[9,0]
        mz = current_state[10,0]
        g = math.sqrt(pow(measurement[6,0],2)+pow(measurement[7,0],2)+pow(measurement[8,0],2))
        m = math.sqrt(pow(mxy,2)+pow(mz,2))
        #Calculate the predicted compass heading
        predicted_measurement[0,0] = mxy*math.cos(w) - mz*math.sin(p)
        predicted_measurement[1,0] = mxy*math.sin(w) - mz*math.sin(r)
        predicted_measurement[2,0] = m - math.sqrt(pow(predicted_measurement[0,0],2)+pow(predicted_measurement[1,0],2))
        #Calculate the predicted gyro readings
        predicted_measurement[3,0] = vr - vw*math.sin(p) + current_state[6,0]
        predicted_measurement[4,0] = vp*math.cos(r)+vw*math.sin(r)*math.cos(p)+current_state[7,0]
        predicted_measurement[5,0] = vw*math.cos(r)*math.cos(p)-vp*math.sin(r) + current_state[8,0]
        #Calculate the predicted accelerometer readings
        predicted_measurement[6,0] = -g*math.sin(p)
        predicted_measurement[7,0] = g*math.sin(r)
        predicted_measurement[8,0] = g-math.sqrt(pow(predicted_measurement[6,0],2)+pow(predicted_measurement[7,0],2))
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
        measurement[8,0] = measurement_msg.linear_acceleration.z
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
        # Ugh, stupid quats, just publish raw for now :-D
        imu_msg.orientation.x = self.kalman_state[0,0]
        imu_msg.orientation.y = self.kalman_state[1,0]
        imu_msg.orientation.z = self.limitTheta(self.kalman_state[2,0])
        imu_msg.orientation.w = 0
        imu_msg.orientation_covariance = list(self.kalman_covariance[0:3,0:3].flatten())
        imu_msg.angular_velocity.x = self.kalman_state[3,0]
        imu_msg.angular_velocity.y = self.kalman_state[4,0]
        imu_msg.angular_velocity.z = self.kalman_state[5,0]
        imu_msg.angular_velocity_covariance = list(self.kalman_covariance[3:6,3:6].flatten())
        imu_msg.linear_acceleration.x = -1#self.kalman_state[6,0]
        imu_msg.linear_acceleration.y = -1#self.kalman_state[7,0]
        imu_msg.linear_acceleration.z = -1#self.kalman_state[8,0]
        #imu_msg.linear_acceleration_covariance = \
                #list(self.kalman_covariance[6:9,6:9].flatten())
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
    rospy.init_node("sf9dof_ukf_euler", log_level=rospy.DEBUG)
    ukf = SF9DOF_UKF()
    ukf.initialize_filter(rospy.Time.now())
    rospy.Subscriber("state", State, ukf.handle_measurement)
    rospy.spin()
