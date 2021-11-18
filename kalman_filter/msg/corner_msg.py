#!/usr/bin/env python

from geometry_msgs.msg import Point

class CornerMsg:

    def __init__(self, p, psi, id, l_a, l_b):
        self.p = p
        self.psi = psi
        self.id = id
        self.l_a = l_a
        self.l_b = l_b

