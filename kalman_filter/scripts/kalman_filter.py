#!/usr/bin/env python

import rospy
import math
import jacobians
import tf.transformations
import numpy as np
from numpy.linalg import inv
from state import State
from geometry_msgs.msg import Pose, Quaternion
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
from kalman.msg import DepthFeatures
from kalman.msg import CornerMsg


state = State(Pose())
state.pose.position.x = 0
state.pose.position.y = 0
state.pose.orientation.x = 0
state.pose.orientation.y = 0
state.pose.orientation.z = 0
state.pose.orientation.w = 0

latest_pose = Pose()
corners = []

measurements_taken = 0
subscribing_to_something = False
left_tick = int()
right_tick = int()
left_prev = int()
right_prev = int()

mean = np.mat('0; 0; 0', float)
covariance = np.mat('1 1 1; 1 1 1; 1 1 1', float)

WHEEL_CIRCUMFERENCE = 0.2073 # m
TICKS_PER_REV = 4096.0
Rw = 0.1435 # m

DURATION = 0.1
LAST_TIMER = 0
DURDIFF = 0


def predict(left_tick, right_tick):
        global state
        global covariance
        global DURATION
        global DURDIFF

        control = get_control(left_tick, right_tick)
        covariance = get_covariance(state, control, covariance)
        state = transition_model(state, control)
        return state, covariance

def measure(corners):
        global state
        global latest_pose
        global mean
        global covariance
        global measurements_taken

        # measurement error matrix
        q = jacobians.getQ()

        predicted_theta = tf.transformations.euler_from_quaternion([ state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w ])[2]
        actual_theta = tf.transformations.euler_from_quaternion([ latest_pose.orientation.x, latest_pose.orientation.y, latest_pose.orientation.z, latest_pose.orientation.w ])[2]

        # loop through each corner
        if not corners:
                return None, None
        for i, corner in enumerate(corners):
                measurements_taken += 1
                r = math.sqrt( (corner.p.x - state.pose.position.x) ** 2 + (corner.p.y - state.pose.position.y) ** 2 )
                phi = math.atan2( corner.p.y - state.pose.position.y, corner.p.x - state.pose.position.x ) - predicted_theta

                dist_to_landmark = math.sqrt( (corner.p.x - latest_pose.position.x) ** 2 + (corner.p.y - latest_pose.position.y) ** 2 )
                angle_to_landmark = phi = math.atan2( corner.p.y - latest_pose.position.y, corner.p.x - latest_pose.position.x ) - actual_theta

                h = jacobians.getH(corner, state.pose, r)
                h_transpose = h.transpose()
                s = h * covariance * h_transpose + q
                k = covariance * h_transpose * inv(s)
                
                z_hat = np.array([ [r], [phi], [i] ],)
                z = np.array([[ dist_to_landmark], [angle_to_landmark], [i] ])
                difference = z - z_hat

                mean = (mean*(measurements_taken-1) + k * difference)/measurements_taken
                covariance *= np.identity(3) - k * h

                return mean, covariance

def display_prediction(predicted_state, predicted_covariance):
        print("\tPREDICTION:")
        #print(" covariance: \n\t" + str(predicted_covariance))
        print(" state:" + 
                "\n\tposition x: " + str(predicted_state.pose.position.x) + ", y: " + str(predicted_state.pose.position.y) +
                "\n\torientation x: " + str(predicted_state.pose.orientation.x) +
                                ", y: " + str(predicted_state.pose.orientation.y) + 
                                ", z: " + str(predicted_state.pose.orientation.z) +
                                ", w: " + str(predicted_state.pose.orientation.w))

def display_measurements(measured_mean, measured_cov):
        print("\tMEASUREMENTS:")
        #print(" covariance: \n\t" + str(measured_cov))
        print(" mean:\n\t" + str(measured_mean))

def kalman_update(timer):
        global corners
        global left_tick
        global right_tick
        global subscribing_to_something
        topics_published = dict(rospy.get_published_topics())
        if subscribing_to_something:
                if '/sensor_state' in topics_published.keys():
                        predicted_state, predicted_cov = predict(left_tick, right_tick)
                        display_prediction(predicted_state, predicted_cov)
                if '/depth_features' in topics_published.keys():
                        measured_mean, measured_cov = measure(corners)
                        display_measurements(measured_mean, measured_cov)
                

def get_covariance(p_state, control, cov):
        g = jacobians.getG(p_state.pose, control)
        g_transpose = g.transpose()

        v = jacobians.getV(p_state.pose, control)
        v_transpose = v.transpose()

        m = jacobians.getM([ .05, .05 ], .05)

        model = g * cov * g_transpose + v * m * v_transpose   
        cov = model.tolist()

        return cov


def transition_model(state, control):
        global Rw

        # get yaw (z rotation euler) from orientation
        theta = tf.transformations.euler_from_quaternion([ state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w ])[2]

        # theta = (dr - dl) / (2Rw)
        theta_prime = (control[0] - control[1]) / (2 * Rw)

        theta = jacobians.displaceAngle(theta, theta_prime)

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        state.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])

        # x = ((dl + dr)/2)cos(theta + (dr - dl)/2Rw)
        state.pose.position.x += ((control[0] + control[1]) / 2) * math.cos(theta + ((control[0] - control[1]) / (2 * Rw)))

        # y = ((dl + dr)/2)sin(theta + (dr - dl)/2Rw)
        state.pose.position.y += ((control[0] + control[1]) / 2) * math.sin(theta + ((control[0] - control[1]) / (2 * Rw)))

        '''
        # calculate velocity
        vl = control[0] / (1.0 / 30.0)
        vr = control[1] / (1.0 / 30.0)

        vx = ((vl + vr) * math.cos(theta)) / 2
        vy = ((vl + vr) * math.sin(theta)) / 2
        vtheta = (vr - vl) / (2 * Rw) 
        '''

        return state


def get_control(left_tick, right_tick):
        global left_prev
        global right_prev
        global TICKS_PER_REV
        global WHEEL_CIRCUMFERENCE

        dl = ((left_tick - left_prev) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
        dr = ((right_tick - right_prev) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE

        left_prev = left_tick
        right_prev = right_tick

        return [ dl, dr ]


def callback(data):
        global left_tick
        global right_tick
        global left_prev
        global right_prev
        global subscribing_to_something
        if not subscribing_to_something:
                print('CALLBACK')
                left_prev = data.left_encoder
                right_prev = data.right_encoder
        subscribing_to_something = True
        
        left_tick = data.left_encoder
        right_tick = data.right_encoder


def set_latest_orientation(data):
        global latest_pose
        global subscribing_to_something
        if not subscribing_to_something:
                print('SET_LATEST_ORIENTATION')
        subscribing_to_something = True

        latest_pose = data.pose.pose

def get_depth_features(data):
        global corners
        global subscribing_to_something
        if not subscribing_to_something:
                print('GET_DEPTH_FEATURES')
        subscribing_to_something = True

        corners = data.corners


def listen():
        global DURATION
        rospy.init_node('kalman_filter', anonymous=False)
        rospy.Subscriber('/sensor_state', SensorState, callback)
        rospy.Subscriber('/odom', Odometry, set_latest_orientation)
        rospy.Subscriber('/depth_features', DepthFeatures, get_depth_features)
        rospy.Timer(rospy.Duration(DURATION), kalman_update)

        # sleep to connect with roscore
        rospy.sleep(.5)

        # prevent node from terminating
        rospy.spin()


if __name__ == '__main__':
        try:
                listen()
        except rospy.ROSInterruptException:
                pass
