#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
import cv2
import pcl
from get_rosbaginfo import get_type_and_topic, get_baginfo
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

class ImageSaver(object):
    def __init__(self, save_path, output_type, bagfile, topic):
        self.save_path = save_path
        self.topic = topic
        self.output_type = output_type
        self.bagfile = bagfile
        self.img_datasets = []

    def img_loader(self, image_msg):
        bridge = CvBridge()
        rospy.loginfo(image_msg.encoding)
        print(image_msg.height)
        camera_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        timestamp = image_msg.header.stamp.secs + ((image_msg.header.stamp.nsecs + 0.0) / 1000000000)
        if self.output_type == "image":
            self.save_image(camera_img, timestamp, '1')
        elif self.output_type == "h5file":
            self.save_h5file(camera_img, timestamp)

    def save_image(self, img, timestamp, sfx):
        cv2.imwrite(self.save_path + '/camera_' + sfx + '_' + "{:.5f}".format(timestamp) + '.png', img)

    def save_h5file(self, img, timestamp):
        print(timestamp)
        a = get_baginfo(self.bagfile)
        print(a[self.topic][1])

    def process(self):
        node_name = "get_%s_and_convert_to_RGB_Image" % self.topic
        rospy.init_node('rosbag_data_extract_unsync', anonymous=True)
        rospy.Subscriber(self.topic, Image, self.img_loader)
        rospy.spin()

def rosbag_data_extract_sample():
    try:
        save_path = sys.argv[1]
        topic = sys.argv[2]
        output_type = "image"
        bagfile = "/home/katou01/.autoware/autoware-201701171120.bag"
    except Exception, e:
        sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")

    image_saver = ImageSaver(save_path, output_type, bagfile, topic)
    image_saver.process()
    # node_name = "get_%s_and_convert_to_RGB_Image" % topic
    # rospy.init_node('rosbag_data_extract_unsync', anonymous=True)
    #
    # rospy.Subscriber(topic, Image, img_loader)
    # rospy.spin()

if __name__ == '__main__':
    rosbag_data_extract_sample()
