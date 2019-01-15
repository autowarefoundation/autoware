#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import *
from geometry_msgs.msg import Point
rospy.init_node("test_occupancy_grid")

p = rospy.Publisher("/occupancy_grid", SimpleOccupancyGridArray)

r = rospy.Rate(1)

def cells(x_offset):
    ret = []
    for i in range(0, 20):
        for j in range(0, 20):
            ret.append(Point(x = 0.05 * i + x_offset, y = 0.05 * j, z = 0))
    return ret


while not rospy.is_shutdown():
    now = rospy.Time.now()
    occupancy_grid_array = SimpleOccupancyGridArray()
    for i in range(10):
        occupancy_grid = SimpleOccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = now
        occupancy_grid.coefficients = [0, 0, 1, i * 0.2]
        occupancy_grid.resolution = 0.05      #5cm resolution
        occupancy_grid.cells = cells(i / 2.0)
        occupancy_grid_array.grids.append(occupancy_grid)
    occupancy_grid_array.header.stamp = now
    occupancy_grid_array.header.frame_id = "map"
    p.publish(occupancy_grid_array)
    r.sleep()    
