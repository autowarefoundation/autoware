#!/usr/bin/env python

import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
import rospy
from tf.transformations import quaternion_from_euler


class BoundingBoxArrayPublisher(object):

    def __init__(self):
        self.seq = 0

        self.frame_id = rospy.get_param('~frame_id')
        if (rospy.has_param('~positions') or
                rospy.has_param('~rotations') or
                rospy.has_param('~dimensions')):
            # Deprecated bounding box pose/dimension specification
            rospy.logwarn("DEPRECATION WARNING: Rosparam '~positions', "
                          "'~rotations' and '~dimensions' are being "
                          "deprecated. Please use '~boxes' instead.")
            positions = rospy.get_param('~positions')
            rotations = rospy.get_param('~rotations')
            dimensions = rospy.get_param('~dimensions')
            if len(rotations) != len(positions):
                rospy.logfatal('Number of ~rotations is expected as {}, but {}'
                            .format(len(positions), len(rotations)))
                sys.exit(1)
            if len(dimensions) != len(positions):
                rospy.logfatal('Number of ~dimensions is expected as {}, but {}'
                            .format(len(positions), len(dimensions)))
                sys.exit(1)
            self.boxes = []
            for pos, rot, dim in zip(positions, rotations, dimensions):
                self.boxes.append({
                    'position': pos,
                    'rotation': rot,
                    'dimension': dim,
                })
        else:
            self.boxes = rospy.get_param('~boxes')
        self.pub = rospy.Publisher('~output', BoundingBoxArray, queue_size=1)

        rate = rospy.get_param('~rate', 1)
        self.timer = rospy.Timer(rospy.Duration(1. / rate), self.publish)

    def publish(self, event):
        bbox_array_msg = BoundingBoxArray()
        bbox_array_msg.header.seq = self.seq
        bbox_array_msg.header.frame_id = self.frame_id
        bbox_array_msg.header.stamp = event.current_real
        for box in self.boxes:
            pos = box['position']
            rot = box.get('rotation', [0, 0, 0])
            qua = quaternion_from_euler(*rot)
            dim = box['dimension']

            bbox_msg = BoundingBox()
            bbox_msg.header.seq = self.seq
            bbox_msg.header.frame_id = self.frame_id
            bbox_msg.header.stamp = event.current_real
            bbox_msg.pose.position = Point(*pos)
            bbox_msg.pose.orientation = Quaternion(*qua)
            bbox_msg.dimensions = Vector3(*dim)

            bbox_array_msg.boxes.append(bbox_msg)

        self.pub.publish(bbox_array_msg)


if __name__ == '__main__':
    rospy.init_node('bounding_box_array_publisher')
    BoundingBoxArrayPublisher()
    rospy.spin()
