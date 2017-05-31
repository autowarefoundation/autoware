#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import cv2
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge

save_path = None

def cloud_loader(msg):
    timestamp = msg.header.stamp.secs + ((msg.header.stamp.nsecs + 0.0) / 1000000000)
    save_pcd(msg, timestamp, save_path)

def save_pcd(cloud, timestamp, path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)), dtype=np.float32)[:, 0:3])
    p.to_file(path + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')

def rosbag_data_extract_sample():
    global save_path
    try:
        save_path = sys.argv[1]
        topic = sys.argv[2]
    except Exception, e:
        #sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")
        save_path = './sample'

    node_name = "get_%s_and_convert_to_PCD_data" % topic
    rospy.init_node('rosbag_pcd_extract_unsync', anonymous=True)

    rospy.Subscriber(topic, PointCloud2, cloud_loader)
    rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract_sample()
