from laser_feature_extraction.msg import LineMsg
from geometry_msgs.msg import Point
import math

class Line:
    
    def __init__(self, A, B, C, p_a, p_b, ID):
        self.A = A
        self.B = B
        self.C = C
        self.p_a = p_a
        self.p_b = p_b
        self.ID = ID
        self.length = -1
        self.slope = -1
        if type(p_a) != type(1) and type(p_b) != type(1):
            self.length = self._getLength(p_a, p_b)
            self.slope = self._getSlope(p_a, p_b)
        self.msg = LineMsg(A, B, C, p_a, p_b, ID)

    def _getLength(self, p_a, p_b):
        return math.sqrt((p_a.x-p_b.x)**2+(p_a.y-p_b.y)**2)

    def _getSlope(self, p_a, p_b):
        if p_b.x-p_a.x != 0:
            return (p_b.y-p_a.y)/(p_b.x-p_a.x)
        return 999
