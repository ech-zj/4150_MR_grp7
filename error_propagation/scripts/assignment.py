#!/usr/bin/env python

import rospy
from Line import Line
from Corner import Corner
from laser_feature_extraction.msg import LineMsg, CornerMsg, DepthFeatures
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
import math
import numpy as np
import jacobians

line_id = 0

pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
pub_features = rospy.Publisher('depth_features', DepthFeatures, queue_size=10)

# error propagation
pose = Pose()
pose.position.x = 0.0
pose.position.y = 0.0
pose.orientation = 0

cov = np.mat('1 1 1; 1 1 1; 1 1 1', float)

qt = np.mat('.05 0 0; 0 .05 0; 0 0 .05', float)

# Make an rviz line list given a list of Line objects
def buildRvizLineList(lines):
    '''
    Creates a LineList Marker object and publishes it to Rviz.
    This function assumes there is a global Publisher created with the following line:
    pub_rviz = rospy.Publisher('visualization_marker', Marker, queue_size=10)
 
    
    Parameters:
        lines (list): list of Line objects to visualize
 
    Returns:
        None
    '''

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


    pub.publish(line_list)
    return line_list

def _getSlope(p_a, p_b):
        if p_b.x-p_a.x != 0 and p_b.y-p_a.y != 0:
            return (p_b.y-p_a.y)/(p_b.x-p_a.x)
        return 0

def getDistBetwPoints(p_a, p_b):
    return math.sqrt(math.pow(p_a.x-p_b.x,2) + math.pow(p_a.y-p_b.y,2))

def getLine(points, first=False):
    THRESHOLD = 0.12
    '''
    Given a list of geometry_msgs/Point objects, return the line that fits over them.

    Parameters:
        points (list): list of Point objects
        first (bool): True if the first time this is called, false otherwise.
                      This is needed so that we can use the point furthest away from the first Point object in the list
                      when this function is called for the first time.

    Returns:
        Line or -1: the Line obect that fits over the points, or -1 indicating that a line could not be found to fit the points.
        float: the max distance found between the line and any point in the points parameter
    '''
    # print('In getLine')

    # Get index of last point in the list
    l = len(points) - 1

    # If it's the first set of points, then don't use first and last points to find the line
    # Instead, use the first point and the point with furthest distance from the first point
    if first:
        # Loop through and find the point furthest away from the first point
        max_d = 0.0
        i_max_d = 0
        for i_p, p in enumerate(points):
            d = getDistBetwPoints(p, points[0])
            # print('points[0]: %s\n p: %s\n d: %s' % (points[0],p,d))
            if d > max_d:
                max_d = d
                i_max_d = i_p
        # Set
        l = i_max_d

    line = getLineBetweenPoints(points[0], points[l])
    max_dist = 0.0
    i_max_dist = 0

    for p_i in range(len(points)):
        temp_max_dist = getDistanceToLine(points[p_i], line)
        if temp_max_dist > max_dist:
            max_dist = temp_max_dist
            i_max_dist = p_i
    if max_dist > THRESHOLD:
        line_temp = Line(0,0,0,-1,0,0)
        return line_temp, i_max_dist
    if max_dist < 0:
        print 'returning a bad distance'
        print max_dist
    return line, max_dist

def getAllLines(points):
    '''
    Given a list of geometry_msgs/Point objects, return all the lines 
    that can be fit over them
 
    Parameters:
        points (list): list of Point objects

    Returns:
        list: list of Line objects that were fit over the points parameter
    '''

    #print('In getAllLines')
    #print('points:')
    #print('%s,\n%s\n...\n%s' % (points[0], points[1], points[-1]))

    # Initialize an empty list to hold all the lines
    result = []

    # Get the first line over all the points
    # Do this outside the loop because the endpoints are different the first time we do this
    l, i_max_dist = getLine(points, True)
    result.append(l)

    #print('First line:')
    #result[0].printLine()

    # Maintain a list of point sets to process (create and evaluate a line for all sets of points in toProcess)
    toProcess = []

    # If the first line wasn't good then split the data and add the sets to toProcess
    if result[0].p_a == -1:
        #print('Line not good, adding to toProcess')

        # Split data on the point with max distance from the line
        p_a = points[0:i_max_dist]
        p_b = points[i_max_dist:]

        # Add new point sets to toProcess
        toProcess.append(p_a)
        toProcess.append(p_b)

        # Remove the bad line
        result = result[:-1]

    # Send points to be displayed in rviz (optional, debugging) 
    #highlightPointSet(p_a,1)
    #highlightPointSet(p_b,2)
    #i_marker = 3


    # while there are still point sets to find lines for
    while len(toProcess) > 0:

        # Get next set of points to find a line for
        points = toProcess[0]

        # Send points to be displayed in rviz (optional, debugging) 
        #highlightPointSet(points, i_marker)
        #i_marker += 1

        # Check that we have more than 2 points. If we only have endpoints then no line should be valid.
        if len(toProcess[0]) > 2:

            #**************************************************
            # Calling getLine
            #**************************************************
            l, i_max_dist = getLine(toProcess[0])
            result.append(l)
            #print('New line:')
            #result[-1].printLine()

            # If the line is invalid, then remove it and split the data
            if result[-1].p_a == -1:
                # Split data
                p_a = points[0:i_max_dist]
                p_b = points[i_max_dist:]

                # Add new sets to toProcess list
                toProcess.append(p_a)
                toProcess.append(p_b)

                # Remove the bad line
                result = result[:-1]

        # Remove the first list in toProcess (the one we just processed)
        toProcess = toProcess[1:]

    #print('Exiting getAllLines')
    return result


def getLineBetweenPoints(p_a, p_b):
    global line_id
    line_id += 1
    #Ax+By+C=0
    A = _getSlope(p_a, p_b)
    B = -1.0
    C = A*-p_a.x+p_a.y
    
    #normalize
    s = math.sqrt((A*A)+(B*B))
    A = A/s
    B = B/s
    C = C/s

    line = Line(A, B, C, p_a, p_b, line_id)
    return line

def getCornersFromLines(lines):
    ANGLE_THRESHOLD = math.pi/4
    SHORT_DISTANCE = 0.25
    corners = []
    for i, line in enumerate(lines):
        j = i+1
        while j < len(lines):
            line2 = lines[j]
            theta = math.atan((line.slope-line2.slope)/(1+line.slope*line2.slope))
            if theta > ANGLE_THRESHOLD:
                line_compare = [line.p_a, line.p_b]
                line2_compare = [line2.p_a, line2.p_b]
                for point in line_compare:
                    for point2 in line2_compare:
                        d = getDistBetwPoints(point, point2)
                        if d <= SHORT_DISTANCE:
                            cornerPoint = Point()
                            cornerPoint.x = (point.x+point2.x)/2
                            cornerPoint.y = (point.y+point2.y)/2
                            new_corner = Corner(cornerPoint,0,0,line,line2)
                            corners.append(new_corner)
            j += 1

    return corners


def getDistanceToLine(point, line):
    relative = abs(point.x*line.A + point.y*line.B + line.C)
    return relative

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

    pub.publish(pointMarker)
    return pointMarker

def getPublishLines(lines):
    pub_lines = []
    for i, line in enumerate(lines):
        lineMsg = LineMsg()
        lineMsg.A = line.A
        lineMsg.B = line.B
        lineMsg.C = line.C
        lineMsg.p_a = line.p_a
        lineMsg.p_b = line.p_b
        lineMsg.id = i
        pub_lines.append(lineMsg)
    return pub_lines

def getPublishCorners(corners):
    pub_corners = []
    for i, corner in enumerate(corners):
        cornerMsg = CornerMsg()
        cornerMsg.p = corner.p
        cornerMsg.psi = corner.psi
        cornerMsg.id = i
        cornerMsg.l_a = corner.l_a
        cornerMsg.l_b = corner.l_b
    return pub_corners

def callback(data):
    global pose
    global cov
    global qt

    angle_min = data.angle_min
    angle_increment = data.angle_increment
    points = []
    for i, r in enumerate(data.ranges):
        point = Point()
        angle = angle_min + i * angle_increment
        point.x = r * math.cos(angle)
        point.y = r * math.sin(angle)
        points.append(point)
    MIN = 0.005
    NEG_MIN = -MIN
    for i in range(len(points)-1,-1,-1):
        p = points[i]
        if NEG_MIN < p.x < MIN and NEG_MIN < p.y < MIN:
            del points[i]
    lines = getAllLines(points)
    buildRvizLineList(lines)
    corners = getCornersFromLines(lines)
    buildRvizCorners(corners)
    depthFeatures = DepthFeatures()
    # I was not able to use return from buildRvizCorners or buildRvizLines
    pub_lines = getPublishLines(lines)
    pub_corners = getPublishCorners(corners)
    depthFeatures.lines = pub_lines
    depthFeatures.corners = pub_corners
    pub_features.publish(depthFeatures)

    # get covariance
    for x in range(len(corners)):
        h = jacobians.getH(corners[x], pose, get_q(corners[x], pose))
        h_transpose = h.transpose()
        
        model = h * cov * h_transpose + qt
        cov = model
        print(model)


def get_q(corner, pose):
    return math.sqrt((corner.p.x - pose.position.x) ** 2 + (corner.p.y - pose.position.y) ** 2)


def main():
    print 'In main'
    rospy.init_node('assignment', anonymous=True)
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.Subscriber('/scan', LaserScan, callback) 
    rospy.spin()

if __name__ == '__main__':
    main()
