#!/usr/bin/env python

import rospy
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
import math
import tf.transformations
from state import State


state = State(0, 0, 0, 0, 0, 0)
pub = rospy.Publisher('my_odom', Odometry, queue_size=10)

left_prev = 0
right_prev = 0

WHEEL_CIRCUMFERENCE = 0.2073 # m
TICKS_PER_REV = 4096.0
Rw = 0.1435 # m


# x: State object, u: control vector
def transition_model(x, u):
        # x = ((dl + dr)/2)cos(theta + (dr - dl)/2Rw)
        x.x += ((u[0] + u[1]) / 2) * math.cos(x.theta + ((u[1] - u[0]) / (2 * Rw)))

        # y = ((dl + dr)/2)sin(theta + (dr - dl)/2Rw)
        x.y += ((u[0] + u[1]) / 2) * math.sin(x.theta + ((u[1] - u[0]) / (2 * Rw)))

        # theta = (dr - dl) / (2Rw)
        theta_prime = (u[1] - u[0]) / (2 * Rw)

        x.theta = displaceAngle(x.theta, theta_prime)

        # calculate velocity
        vl = u[0] / (1.0 / 30.0)
        vr = u[1] / (1.0 / 30.0)

        x.vx = ((vl + vr) * math.cos(x.theta)) / 2
        x.vy = ((vl + vr) * math.sin(x.theta)) / 2
        x.vtheta = (vr - vl) / 2 * Rw 

        return x


def findDistanceBetweenAngles(a, b):
    '''
    Get the smallest orientation difference in range [-pi,pi] between two angles 
    Parameters:
        a (double): an angle
        b (double): an angle
    
    Returns:
        double: smallest orientation difference in range [-pi,pi]
    '''
    result = 0
    difference = b - a
    
    if difference > math.pi:
      difference = math.fmod(difference, math.pi)
      result = difference - math.pi
 
    elif(difference < -math.pi):
      result = difference + (2*math.pi)
 
    else:
      result = difference
 
    return result
 
 
 
def displaceAngle(a1, a2):
    '''
    Displace an orientation by an angle and stay within [-pi,pi] range
    Parameters:
        a1 (double): an angle
        a2 (double): an angle
    
    Returns:
        double: The resulting angle in range [-pi,pi] after displacing a1 by a2
    '''
    a2 = a2 % (2.0*math.pi)
 
    if a2 > math.pi:
        a2 = (a2 % math.pi) - math.pi
 
    return findDistanceBetweenAngles(-a1,a2)


def callback(data):
        global left_prev
        global right_prev
        global TICKS_PER_REV
        global WHEEL_CIRCUMFERENCE
        global Rw
        global state
        global pub

        left = data.left_encoder
        right = data.right_encoder

        # compute distance traveled by each wheel

        dl = ((left - left_prev) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
        dr = ((right - right_prev) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE

        left_prev = left
        right_prev = right

        print("dl = " + str(dl) + ", dr = " + str(dr))

        # set u
        u = [dl, dr]

        x_prime = transition_model(state, u)
        state = x_prime

        # display new theta
        print("theta = " + str(state.theta))

        # display x and y position
        print("x = " + str(state.x) + ", y = " + str(state.y))

        # display velocity
        print("vx = " + str(state.vx) + ", vy = " + str(state.vy) + ", vtheta = " + str(state.vtheta))

        # build odometry message
        odom_msg = build_odom_msg(state, Odometry())
        pub.publish(odom_msg)


def build_odom_msg(state, odom):
        # set pose

        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0

        q = tf.transformations.quaternion_from_euler(0, 0, state.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set twist

        odom.twist.twist.linear.x = state.vx
        odom.twist.twist.linear.y = state.vy
        odom.twist.twist.linear.z = 0

        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = state.vtheta

        return odom


def listen():
        rospy.init_node('odom_compute', anonymous=False)
        rospy.Subscriber('/sensor_state', SensorState, callback)

        # sleep to connect with roscore
        rospy.sleep(.5)

        # prevent node from terminating
        rospy.spin()


if __name__ == '__main__':
        try:
                listen()
        except rospy.ROSInterruptException:
                pass

