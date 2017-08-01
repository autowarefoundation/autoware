#!/usr/bin/python

import os
import sys
import sdk
import rospy
from tf import transformations
from tf import TransformBroadcaster
import numpy as np
from player import PosePlayer

from nav_msgs.msg import Path


if __name__ == '__main__':
    datasetDir = sys.argv[1]
    rospy.init_node('oxford_path_pub', anonymous=True)
    pathPub = rospy.Publisher('/oxford/path', Path, queue_size=1)
    
    dataset = sdk.Dataset(datasetDir)
    poses = dataset.getIns()
    vPath = Path()
    vPath.header.frame_id = 'world'
#     vPath.header.stamp = rospy.Time.now()
    
    for pose in poses:
        npose = PosePlayer.createPoseFromRPY (
            pose[1], pose[2], pose[3], pose[4], pose[5], pose[6])
        npose.header.frame_id = 'world'
        vPath.poses.append(npose)
        
    rt = rospy.Rate(1.0)
    while (not rospy.is_shutdown()):
        vPath.header.stamp = rospy.Time.now()
        pathPub.publish(vPath)
        rt.sleep()
