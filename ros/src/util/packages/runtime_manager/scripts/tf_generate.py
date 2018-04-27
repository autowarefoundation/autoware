#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from subprocess import call

def callback(data):
    points = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
    counter = len(list(points))

    coordinate = [0, 0, 0]
    for index in range(len(coordinate)):
        points = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
        coordinate[index] = -1*(sum(map(lambda x: x[index], points)) / counter)

    sys.stderr.write("world_to_map tf is running. params: %f %f %f\n" %(coordinate[0], coordinate[1], coordinate[2]))
    call(["rosrun", "tf", "static_transform_publisher", "__name:=world_to_map", "%f"%(coordinate[0]), "%f"%(coordinate[1]), "%f"%(coordinate[2]), "0", "0", "0", "/world", "/map", "10"])

    rospy.signal_shutdown("end")

def listener():
    rospy.init_node('listener', anonymous =True)
    rospy.Subscriber("points_map", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
