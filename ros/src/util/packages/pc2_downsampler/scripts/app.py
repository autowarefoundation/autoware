#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String, Header
import message_filters
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Point32
import math
import time
import pcl

import pprint
pp = pprint.PrettyPrinter(indent=2)


class PC2DownSampler(object):
    def __init__(self):
        self.__previous_time = time.time()
        self.__period = None  # [sec]
        self.__frame_id = None
        self.__leaf_size = None

    def callback(self, raw_data, republisher):
        current_time = time.time()
        if self.__period < current_time - self.__previous_time:
            self.__previous_time += (1+int((current_time - self.__previous_time)/self.__period)) * self.__period

            points = []
            for point in pc2.read_points(raw_data):
                points.append(list(point[0:3]))
            points_map_pcl = pcl.PointCloud(points)
            pc_filter = points_map_pcl.make_voxel_grid_filter()
            pc_filter.set_leaf_size(*self.__leaf_size)
            points_map_pcl = pc_filter.filter()

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.__frame_id
            downsampled_data = pc2.create_cloud_xyz32(header, points_map_pcl.to_list())

            republisher.publish(downsampled_data)

    def setup(self, topic_name, period=1.0, frame_id="world", leaf_size=(10.0, 10.0, 10.0), latch=False):
        self.__period = period
        self.__frame_id = frame_id
        self.__leaf_size = leaf_size
        topic_name = topic_name if topic_name[0] != "/" else topic_name[1:]
        pc2_subscriber = message_filters.Subscriber('/{}'.format(topic_name), PointCloud2)
        pc2_republisher = rospy.Publisher('/downsampled_{}'.format(topic_name), PointCloud2, queue_size=3, latch=latch)

        pc2_subscriber.registerCallback(self.callback, pc2_republisher)


if __name__ == '__main__':
    rospy.init_node("pc2_downsampler", anonymous=True)
    PC2DownSampler().setup("/points_map", 1.5, frame_id="map", leaf_size=(10.0, 10.0, 25.0), latch=True)
    PC2DownSampler().setup("/points_raw", 1.0, frame_id="velodyne", leaf_size=(0.5, 0.5, 0.5))
    rospy.spin()
