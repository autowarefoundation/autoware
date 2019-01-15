#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from dynamic_reconfigure.server import Server
import rospy
from geometry_msgs.msg import PoseArray
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import PeoplePoseArray
from jsk_rviz_plugins.cfg import ClassificationResultVisualizerConfig
from posedetection_msgs.msg import ObjectDetection
import message_filters as MF
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ClassificationResultVisualizer(ConnectionBasedTransport):
    def __init__(self):
        super(ClassificationResultVisualizer, self).__init__()
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.srv = Server(ClassificationResultVisualizerConfig,
                          self.config_callback)
        self.pub_marker = self.advertise("~output", MarkerArray, queue_size=10)

    def subscribe(self):
        sub_cls = MF.Subscriber(
            "~input/classes", ClassificationResult, queue_size=1)
        sub_box = MF.Subscriber(
            "~input/boxes", BoundingBoxArray, queue_size=1)
        sub_pose = MF.Subscriber(
            "~input/poses", PoseArray, queue_size=1)
        sub_people = MF.Subscriber(
            "~input/people", PeoplePoseArray, queue_size=1)
        sub_od = MF.Subscriber(
            "~input/ObjectDetection", ObjectDetection, queue_size=1)
        sync_box = MF.TimeSynchronizer([sub_box, sub_cls], self.queue_size)
        sync_box.registerCallback(self.box_msg_callback)
        sync_pose = MF.TimeSynchronizer([sub_pose, sub_cls], self.queue_size)
        sync_pose.registerCallback(self.pose_msg_callback)
        sync_people = MF.TimeSynchronizer([sub_people, sub_cls], self.queue_size)
        sync_people.registerCallback(self.people_msg_callback)
        sync_od = MF.TimeSynchronizer([sub_od, sub_cls], self.queue_size)
        sync_od.registerCallback(self.od_msg_callback)
        self.syncs = [sync_box, sync_pose, sync_people, sync_od]
        self.subscribers = [sub_cls, sub_box, sub_pose, sub_people, sub_od]

    def unsubscribe(self):
        for sub in self.subscribers:
            sub.unregister()
        self.syncs = []

    def config_callback(self, config, level):
        self.text_color = {'r': config.text_color_red,
                           'g': config.text_color_green,
                           'b': config.text_color_blue,
                           'a': config.text_color_alpha}
        self.text_offset = [config.text_offset_x,
                            config.text_offset_y,
                            config.text_offset_z]
        self.text_size = config.text_size
        self.marker_lifetime = config.marker_lifetime
        return config

    def pose_msg_callback(self, pose, classes):
        bboxes = BoundingBoxArray(header=pose.header)
        for p in pose.poses:
            b = BoundingBox(header=pose.header)
            b.pose = p
            bboxes.boxes.append(b)
        self.box_msg_callback(bboxes, classes)

    def people_msg_callback(self, people, classes):
        bboxes = BoundingBoxArray(header=people.header)
        for p in people.poses:
            b = BoundingBox()
            for i, n in enumerate(p.limb_names):
                if n in ["Neck", "Nose", "REye", "LEye", "REar", "LEar"]:
                    b.header = people.header
                    b.pose = p.poses[i]
                    break
            if not b.header.frame_id:
                b.header = people.header
                b.pose = b.poses[0]
        self.box_msg_callback(bboxes, classes)

    def od_msg_callback(self, od, classes):
        bboxes = BoundingBoxArray(header=od.header)
        for obj in od.objects:
            b = BoundingBox()
            b.pose = obj.pose
        self.box_msg_callback(bboxes, classes)

    def box_msg_callback(self, bboxes, classes):
        msg = MarkerArray()
        for i, data in enumerate(zip(bboxes.boxes, zip(classes.label_names, classes.label_proba))):
            bbox, cls = data
            text = "%s (%.3f)" % cls

            m = Marker(type=Marker.TEXT_VIEW_FACING,
                       action=Marker.MODIFY,
                       header=bbox.header,
                       id=i,
                       pose=bbox.pose,
                       color=ColorRGBA(**self.text_color),
                       text=text,
                       ns=classes.classifier,
                       lifetime=rospy.Duration(self.marker_lifetime))
            m.scale.z = self.text_size
            m.pose.position.x += self.text_offset[0]
            m.pose.position.y += self.text_offset[1]
            m.pose.position.z += self.text_offset[2]
            msg.markers.append(m)

        self.pub_marker.publish(msg)


if __name__ == '__main__':
    rospy.init_node("classification_result_visualizer")
    viz = ClassificationResultVisualizer()
    rospy.spin()
