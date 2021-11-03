#!/usr/bin/env python

import rospy
from Line import Line
from Corner import Corner
from laser_feature_extraction.msg import LineMsg, CornerMsg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

# Make an rviz line list given a list of Line objects
def buildRvizLineList(lines):

    line_list = Marker()
    line_list.header.frame_id = 'base_scan'
    line_list.header.stamp = rospy.Time(0)
    line_list.ns = ''

    line_list.id = 0
    line_list.type = 5
    line_list.action = 0
    
    line_list.scale.x = 0.02

    line_list.color.g = 1.0
    line_list.color.a = 1.0

    # Add the line endpoints to list of points
    for l in lines:
        line_list.points.append(l.p_a)
        line_list.points.append(l.p_b)
    
    return line_list

def buildRvizCorners(corners):

    pointMarker = Marker()
    pointMarker.header.frame_id = 'base_scan'
    pointMarker.header.stamp = rospy.Time(0)
    pointMarker.ns = ''

    pointMarker.id = 10
    pointMarker.type = 8
    pointMarker.action = 0

    pointMarker.scale.x = 0.2
    pointMarker.scale.y = 0.2
    pointMarker.scale.z = 0.2

    pointMarker.color.b = 1.0
    pointMarker.color.a = 1.0
    pointMarker.colors.append(pointMarker.color)


    for c in corners:
        pointMarker.points.append(c.p)

    #pub_rviz.publish(pointMarker)
    return pointMarker

def callback(data):
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    points = []
    for i, r in enumerate(data.ranges):
        point = Point()
        angle = angle_min + i * angle_increment
        point.x = r * math.cos(angle)
        point.y = r * math.sin(angle)
        points.append(point)
    corners = []
    for i in range(1,len(points)-1):
        p_a = points[i-1]
        p_b = points[i]
        p_c = points[i+1]
        line_a = Line(0, 0, 0, p_a, p_b, 0)
        line_b = Line(0, 0, 0, p_b, p_c, 0)
        line_list = buildRvizLineList([line_a, line_b])
        pub.publish(line_list)
        corner = Corner(p_b, 0, 0, line_a, line_b)
        corners.append(corner)
    for i in range(len(corners)//2):
        corner_a = corners[i*2]
        corner_b = corners[i*2+1]
        corner_list = buildRvizCorners([corner_a, corner_b])
        pub.publish(corner_list)


def main():
    print 'In main'
    rospy.init_node('lines_and_corners', anonymous=True)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.Subscriber('/scan', LaserScan, callback) 
    rospy.spin()

if __name__ == '__main__':
    main()
