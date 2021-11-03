#!/usr/bin/env python

import numpy as np

class State:

        def __init__(self, x, y, theta, vx, vy, vtheta):
                self.x = x
                self.y = y
                self.theta = theta
                self.vx = vx
                self.vy = vy
                self.vtheta = vtheta
                self.cov = []

