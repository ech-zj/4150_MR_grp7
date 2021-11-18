#!/usr/bin/env python
import math
import numpy
import tf

###############################################################
# Turtlebot3 hardware data
# TB3 encoders have 12-bit resolution so 4096 ticks per revolution
# TB3 Rw = 143.5mm = 0.1435m
TICKS_PER_REV = 4096.0
WHEEL_CIRCUMFERENCE = 0.2073 #m
Rw = 0.1435 #m
###############################################################


def findDistanceBetweenAngles(a, b):
    
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

    a2 = a2 % (2.0*math.pi)

    if a2 > math.pi:
        a2 = (a2 % math.pi) - math.pi

    return findDistanceBetweenAngles(-a1,a2)



def getG(x, u):
    '''
    Get the transition model matrix for the previous state and latest control

    Parameters:
        x (Pose): The previous pose
        u (numpy.mat): The latest control input
    
    Returns:
        numpy.mat: Matrix representing transition model
    '''
    # Get displacement and orientation change
    d = (u[0] + u[1]) / 2.0
    dtheta = (u[1] - u[0]) / (2.0*Rw)
    
    # Get theta 
    theta = tf.transformations.euler_from_quaternion([x.orientation.x, x.orientation.y, x.orientation.z, x.orientation.w])[2]

    # Create the matrix
    G = numpy.matrix('1 0 -1; 0 1 -1; 0 0 1', float)
    G[0,2] = -d*math.sin(displaceAngle(theta,dtheta))
    G[1,2] = d*math.cos(displaceAngle(theta,dtheta))

    return G

def getV(x, u):
    '''
    Get the motion model matrix for the previous state and latest control

    Parameters:
        x (Pose): The previous pose
        u (numpy.mat): The latest control input vector

    Returns:
        numpy.mat: Matrix representing Jacobian from control to state
    '''


    # Get displacement and orientation change
    d = (u[0] + u[1]) / 2.0
    dtheta = (u[1] - u[0]) / (2.0*Rw)
    
    # Get theta
    theta = tf.transformations.euler_from_quaternion([x.orientation.x, x.orientation.y, x.orientation.z, x.orientation.w])[2]


    # Create the matrix
    V = numpy.matrix('-1 -1; -1 -1; 0 1', float)
    V[0,0] = ( (-d*math.sin(displaceAngle(theta, dtheta))) / (2.0*Rw)) + ( math.cos(displaceAngle(theta,dtheta)) / 2.0)

    V[0,1] = (d*math.sin(displaceAngle(theta, dtheta))  / (2.0*Rw)) + ( math.cos(displaceAngle(theta,dtheta)) / 2.0)

    V[1,0] = (d*math.cos(displaceAngle(theta, dtheta))  / (2.0*Rw)) + ( math.sin(displaceAngle(theta,dtheta)) / 2.0)

    V[1,1] = (-d*math.cos(displaceAngle(theta, dtheta))  / (2.0*Rw)) + ( math.sin(displaceAngle(theta,dtheta)) / 2.0)
    
    V[2,0] = (1.0 / (2.0*Rw))

    V[2,1] = (-1.0/(2.0*Rw))

    return V



def getH(corner, meanPose, q):
    '''
    Obtain the Jacobian matrix for measurement model 

    Parameters:
        corner (CornerMsg): a single CornerMsg object
        meanPose (Pose): the latest pose estimate
        q (float): distance between corner position and the latest pose estimate

    Returns:
        numpy.mat: matrix representing mapping from feature information to robot state
    '''

    H = numpy.mat('0.0 0.0 0.0; 0.0 0.0 -1.0; 0.0 0.0 0.0')
    H[0,0] = -( corner.p.x - meanPose.position.x   / math.sqrt(q))
    H[0,1] = -( corner.p.y - meanPose.position.y   / math.sqrt(q))

    return H

def getM(u, alpha):
    '''
    Obtain the Motion error matrix for motion model 

    Parameters:
        u (list): 2-element list containing left wheel variance and right wheel 
variance
        alpha (double): variance for wheel distance

    Returns:
        numpy.mat: matrix representing the amount of error that can arise from the 
given motion
    '''

    M = numpy.mat('0.0 0.0; 0.0 0.0')

    a_l = u[0]
    a_r = u[1]

    M[0,0] = alpha*u[0]
    M[1,1] = alpha*u[1]

    return M


def getQ():
        return np.mat('.05 0 0; 0 .05 0; 0 0 .5', float)
