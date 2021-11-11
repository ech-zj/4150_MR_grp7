#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState
from nav_msgs.msg import Odometry
import tf.transformations

odom = None
        

def turn_to(theta):
        global odom
        
        # create publisher to change velocity
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        turned = False

        rate = rospy.Rate(60)
        while not turned:
                # get current orientation
                orientation = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
                print(orientation)

                # build message
                twist = Twist()
                twist.angular.z = .4

                # compare to target orientation and stop if true
                if orientation[2] >= theta:
                        turned = True
                        twist.angular.z = 0                

                pub.publish(twist)
                rate.sleep()


def drive_dist(distance):
        global odom

        # create publisher to change velocity
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        position = odom.pose.pose.position
        driven = False
        
        rate = rospy.Rate(60)
        while not driven:
                # get position and use pythagorean theorem to get straight-line distance
                new_position = odom.pose.pose.position
                current_dist = math.sqrt((new_position.x - position.x) ** 2 + (new_position.y - position.y) ** 2)
                print("current dist: " + str(current_dist))

                # build message
                twist = Twist()
                twist.linear.x = .2

                if current_dist >= distance: # stop driving
                        driven = True
                        twist.linear.x = 0

                pub.publish(twist)
                rate.sleep()
        

def callback(data):
        global odom
        odom = data

        # print relevant odom data
        print(
                "\t\tposition:\n" +
                "\t\t\tx: " + str(data.pose.pose.position.x) + "\n" +
                "\t\t\ty: " + str(data.pose.pose.position.y) + "\n" +
                "\t\t\tz: " + str(data.pose.pose.position.z) + "\n" +
                "\t\torientation:\n" +
                "\t\t\tx: " + str(data.pose.pose.orientation.x) + "\n" +
                "\t\t\ty: " + str(data.pose.pose.orientation.y) + "\n" +
                "\t\t\tz: " + str(data.pose.pose.orientation.z) + "\n" +
                "\t\t\tw: " + str(data.pose.pose.orientation.w) + "\n" +
                "\t\tlinear:\n" +
                "\t\t\tx: " + str(data.twist.twist.linear.x) + "\n" +
                "\t\t\ty: " + str(data.twist.twist.linear.y) + "\n" +
                "\t\t\tz: " + str(data.twist.twist.linear.z) + "\n" +
                "\t\tangular:\n" +
                "\t\t\tx: " + str(data.twist.twist.angular.x) + "\n" +
                "\t\t\ty: " + str(data.twist.twist.angular.y) + "\n" +
                "\t\t\tz: " + str(data.twist.twist.angular.z) + "\n"
        )


def listen():
        rospy.init_node("drive_robot", anonymous=False)
        rospy.Subscriber("my_odom", Odometry, callback)
        rospy.Subscriber("/sensorstate", SensorState, get_sensor_state)

        # sleep to connect with roscore
        rospy.sleep(.5)

        turn_to(math.pi / 2)
        drive_dist(.5)

        # prevent terminating
        rospy.spin()


if __name__ == "__main__":
        try:
                listen()
        except rospy.ROSInterruptException:
                pass
