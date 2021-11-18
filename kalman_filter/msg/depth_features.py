#!/usr/bin/env python

from std_msgs.msg import Header
from kalman.msg import CornerMsg
from kalman.msg import LineMsg

class DepthFeatures:
        
        def __init__(self, header, lines, corners):
                self.header = header
                self.lines = lines
                self.corners = corners
