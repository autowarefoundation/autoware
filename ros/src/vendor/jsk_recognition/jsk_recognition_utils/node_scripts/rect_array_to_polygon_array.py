#!/usr/bin/env python

import dynamic_reconfigure.server
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import RectArray
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_utils.cfg import PolygonArrayToPolygonConfig
import rospy


class RectArrayToPolygonArray(ConnectionBasedTransport):

    def __init__(self):
        super(RectArrayToPolygonArray, self).__init__()
        self.pub = self.advertise('~output', PolygonArray, queue_size=1)
        dynamic_reconfigure.server.Server(PolygonArrayToPolygonConfig,
                                          self._config_callback)

    def subscribe(self):
        self.sub = rospy.Subscriber('~input', RectArray, self._convert)

    def unsubscribe(self):
        self.sub.unregister()

    def _config_callback(self, config, level):
        self.index = config.index
        return config

    def _convert(self, msg):
        polys_msg = PolygonArray()
        polys_msg.header = msg.header
        for rect in msg.rects:
            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            pt0 = Point32(x=rect.x, y=rect.y)
            pt1 = Point32(x=rect.x + rect.width, y=rect.y + rect.height)
            poly_msg.polygon.points.append(pt0)
            poly_msg.polygon.points.append(pt1)
            polys_msg.polygons.append(poly_msg)
        self.pub.publish(polys_msg)


if __name__ == '__main__':
    rospy.init_node('rect_array_to_polygon_array')
    RectArrayToPolygonArray()
    rospy.spin()
