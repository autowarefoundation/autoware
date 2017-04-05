#!/usr/bin/python2.7


import rospy
import orbndt
from nav_msgs.msg import Path
import sys
from geometry_msgs.msg import \
    Pose as gPose, \
    PoseStamped
from std_msgs.msg import Header


def createGeomPose (pose):
    cgPose = gPose()
    cgPose.position.x = pose.x
    cgPose.position.y = pose.y
    cgPose.position.z = pose.z
    cgPose.orientation.x = pose.qx
    cgPose.orientation.y = pose.qy
    cgPose.orientation.z = pose.qz
    cgPose.orientation.w = pose.qw
    cHeader = Header()
    cHeader.frame_id = 'world'
    cHeader.stamp = rospy.Time.from_sec(pose.timestamp)
    return cHeader, cgPose


if __name__ == '__main__' :
    
    print ("Loading ground truth")
    groundTruth = orbndt.PoseTable.loadFromBagFile (sys.argv[1], 'world', 'camera1')
    print ("Done Loading")
    
    rospy.init_node('GroundTruthPublisher', anonymous=True)
    pathPub = rospy.Publisher ('/GroundTruth', Path, queue_size=1)
    rate = rospy.Rate (1.0)

    groundTruthPath = Path()
    groundTruthPath.header.stamp = rospy.Time.now()
    groundTruthPath.header.frame_id = 'world'

    for pose in groundTruth.table:
        cHeader, cgPose = createGeomPose(pose)
        cMsg = PoseStamped()
        cMsg.header = cHeader
        cMsg.pose = cgPose
        groundTruthPath.poses.append(cMsg)
    
    while not rospy.is_shutdown():
        pathPub.publish(groundTruthPath)
        print ('Painted')
        rate.sleep()
        