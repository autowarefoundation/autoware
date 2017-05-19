#!/usr/bin/env python
import sys
import os
import rospy
import numpy as np
import cv2
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

save_path = None

def img_loader_1(image_msg):
    bridge = CvBridge()
    camera_img_1 = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    timestamp = image_msg.header.stamp.secs + ((image_msg.header.stamp.nsecs + 0.0) / 1000000000)
    save_image(camera_img_1, timestamp, save_path, '1')

def cloud_loader(msg):
    timestamp = msg.header.stamp.secs + ((msg.header.stamp.nsecs + 0.0) / 1000000000)
    save_pcd(msg, timestamp, save_path)

def save_pcd(cloud, timestamp, path):
    p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)), dtype=np.float32)[:, 0:3])
    p.to_file(path + '/pcd' + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')

def save_image(img, timestamp, path, sfx):
    cv2.imwrite(path + '/camera' + sfx + '/camera_' + sfx + '_' + "{:.5f}".format(timestamp) + '.png', img)

def rosbag_data_extract_sample():
    global save_path
    try:
        save_path = sys.argv[1]
    except Exception, e:
        #sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")
        save_path = './sample'

    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(save_path + '/camera1'):
        os.makedirs(save_path + '/camera1')
    if not os.path.exists(save_path + '/pcd'):
        os.makedirs(save_path + '/pcd')

    rospy.init_node('rosbag_data_extract_unsync', anonymous=True)

    rospy.Subscriber("/camera1/image_raw", Image, img_loader_1)
    rospy.Subscriber("/points_raw", PointCloud2, cloud_loader)
    rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract_sample()
