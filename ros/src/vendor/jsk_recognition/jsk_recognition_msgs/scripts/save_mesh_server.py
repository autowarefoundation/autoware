#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.srv import SaveMesh
from jsk_recognition_msgs.srv import SaveMeshRequest
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class SaveMeshServer(object):

    def __init__(self):
        self.ground_frame_id = rospy.get_param('~ground_frame_id', '')
        self.sub_bbox = rospy.Subscriber('~input/bbox', BoundingBox, self._cb)
        self.srv_client = rospy.ServiceProxy('~save_mesh', SaveMesh)
        self.srv_server = rospy.Service('~request', Empty, self._request_cb)
        self.bbox_msg = None

    def _cb(self, bbox_msg):
        self.bbox_msg = bbox_msg

    def _request_cb(self, req):
        if self.bbox_msg is None:
            rospy.logerr('No bounding box is set, so ignoring the request.')
            return EmptyResponse()

        req = SaveMeshRequest()
        req.box = self.bbox_msg
        req.ground_frame_id = self.ground_frame_id
        self.srv_client.call(req)
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('save_mesh_server')
    server = SaveMeshServer()
    rospy.spin()
