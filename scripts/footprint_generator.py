#! /usr/bin/env python3

from geometry_msgs.msg import Point32, Polygon
from math import cos, sin, pi


class FootprintGenerator:
    def __init__(self, rad):
        """Initialize the FootprintGenerator Object."""
        self.botx = 0
        self.boty = 0
        self.radius = rad
        pass

    # params: x: bot pose in map coordinates
    # params: y: bot pose in map coordinates
    def setPose(self, x, y):
        self.botx = x
        self.boty = y

    # return: a Polygon representing the robots footprint
    def getFootprint(self):
        footprint = Polygon()
        theta = 0.0
        for i in range(10):
            pt = Point32()
            pt.x = self.botx + self.radius * cos(theta)
            pt.y = self.boty + self.radius * sin(theta)
            footprint.points.append(pt)
            theta += (2 * pi) / 10.0
        return footprint
