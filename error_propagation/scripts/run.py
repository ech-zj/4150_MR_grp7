#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def listen():
        rospy.init_node('run')
        rospy.Subscriber('my_odom', Odometry, callback)

        rospy.sleep(.5)

        rospy.spin()

def callback(data):
        print(data)

if __name__ == '__main__':
        listen()
