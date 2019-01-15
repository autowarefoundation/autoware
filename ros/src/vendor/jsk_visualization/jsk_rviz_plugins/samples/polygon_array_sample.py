#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Header
from math import sin, cos, pi
import numpy as np
def SquarePolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x=1.0, y=1.0),
                        Point32(x=-1.0, y=1.0),
                        Point32(x=-1.0, y=-1.0),
                        Point32(x=1.0, y=-1.0)]
    return p
    
def RectanglePolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x=-1.0, y=1.0, z=1.0),
                        Point32(x=-2.0, y=1.0, z=1.0),
                        Point32(x=-2.0, y=-1.0, z=1.0),
                        Point32(x=-1.0, y=-1.0, z=1.0)]
    return p
def CirclePolygon(header):
    p = PolygonStamped()
    p.header = header
    for i in range(100):
        theta = i / 100.0 * 2.0 * pi
        x = 1.0 * cos(theta) + 3.0
        y = 1.0 * sin(theta)
        p.polygon.points.append(Point32(x=x, y=y))
    return p
    
    # star
def StarPolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x= .0000, y= 1.0000 + 3.0),
                        Point32(x= .2245, y= .3090 + 3.0),
                        Point32(x= .9511, y= .3090 + 3.0),
                        Point32(x= .3633, y= -.1180 + 3.0),
                        Point32(x= .5878, y= -.8090 + 3.0),
                        Point32(x= .0000, y= -.3820 + 3.0),
                        Point32(x= -.5878, y= -.8090 + 3.0),
                        Point32(x= -.3633, y= -.1180 + 3.0),
                        Point32(x= -.9511, y= .3090 + 3.0),
                        Point32(x= -.2245, y= .3090 + 3.0)]
    return p
    
if __name__ == "__main__":
    rospy.init_node("polygon_array_sample")
    pub = rospy.Publisher("~output", PolygonArray)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = PolygonArray()
        header = Header()
        header.frame_id = "world"
        header.stamp = rospy.Time.now()
        msg.header = header
        msg.polygons = [SquarePolygon(header),
                        RectanglePolygon(header),
                        CirclePolygon(header),
                        StarPolygon(header)]
        msg.labels = [0, 1, 2, 3]
        msg.likelihood = [np.random.ranf(), np.random.ranf(), np.random.ranf(), np.random.ranf()]
        pub.publish(msg)
