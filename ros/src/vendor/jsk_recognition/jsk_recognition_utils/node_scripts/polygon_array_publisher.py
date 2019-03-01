#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from jsk_recognition_msgs.msg import PolygonArray


class PolygonArrayPublisher(object):
    def __init__(self):
        publish_rate = rospy.get_param("~publish_rate", 1.0)

        frame_id = rospy.get_param("~frame_id")
        param = rospy.get_param("~polygons")
        self.msg = self.parse_params(param, frame_id)

        self.pub_polygons = rospy.Publisher("~output", PolygonArray, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1. / publish_rate), self.publish)

    def parse_params(self, param, frame_id):
        # validate params
        assert isinstance(param, list), "polygons must be list"

        has_label = any("label" in p for p in param)
        has_likelihood = any("likelihood" in p for p in param)

        for polygon in param:
            assert "points" in polygon and isinstance(polygon["points"], list),\
                   "each polygon must have at least 1 'points'"
            for point in polygon["points"]:
                assert len(point) == 3,\
                       "element of 'points' must be list of 3 numbers"
            if has_label:
                assert "label" in polygon, "missing 'label'"
            if has_likelihood:
                assert "likelihood" in polygon, "missing 'likelihood'"

        # parse params
        msg = PolygonArray()
        msg.header.frame_id = frame_id
        for polygon in param:
            ps = PolygonStamped()
            ps.header.frame_id = frame_id
            ps.polygon.points = [Point32(*v) for v in polygon["points"]]
            msg.polygons.append(ps)
            if has_label:
                msg.labels.append(polygon["label"])
            if has_likelihood:
                msg.likelihood.append(polygon["likelihood"])

        return msg

    def publish(self, event):
        # update timestamp
        self.msg.header.stamp = event.current_real
        for p in self.msg.polygons:
            p.header.stamp = event.current_real

        self.pub_polygons.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node("polygon_array_publisher")
    p = PolygonArrayPublisher()
    rospy.spin()
