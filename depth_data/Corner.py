from laser_feature_extraction.msg import CornerMsg, LineMsg
from geometry_msgs.msg import Point

class Corner:

    def __init__(self, p, psi, id, l_a, l_b):
        self.p = p
        self.psi = psi
        self.id = id
        self.l_a = l_a
        self.l_b = l_b
        self.msg = CornerMsg(p, psi, id, l_a, l_b)


