#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_recognition_msgs.msg import ClusterPointIndices
from jsk_topic_tools import ConnectionBasedTransport
import message_filters
import rospy


class AddClusterIndices(ConnectionBasedTransport):
    def __init__(self):
        super(AddClusterIndices, self).__init__()
        self._pub = self.advertise('~output', ClusterPointIndices,
                                   queue_size=1)

    def subscribe(self):
        topics = rospy.get_param('~topics')
        self.subs = []
        for tp in topics:
            sub = message_filters.Subscriber(tp, ClusterPointIndices)
            self.subs.append(sub)
        use_async = rospy.get_param('~approximate_sync', False)
        queue_size = rospy.get_param('~queue_size', 100)
        if use_async:
            slop = rospy.get_param('~slop', 0.1)
            sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size, slop)
        else:
            sync = message_filters.TimeSynchronizer(self.subs, queue_size)
        sync.registerCallback(self._decompose)

    def unsubscribe(self):
        for sub in self.subs:
            sub.unregister()

    def _decompose(self, *cpi_msgs):
        out_msg = ClusterPointIndices()
        out_msg.header = cpi_msgs[0].header
        for cpi_msg in cpi_msgs:
            out_msg.cluster_indices.extend(cpi_msg.cluster_indices)
        self._pub.publish(out_msg)


if __name__ == '__main__':
    rospy.init_node('add_cluster_indices')
    app = AddClusterIndices()
    rospy.spin()
