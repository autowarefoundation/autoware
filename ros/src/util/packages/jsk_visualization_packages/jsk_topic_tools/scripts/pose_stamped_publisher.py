#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def usage():
    "print usage"
    print "Usage: pose_stamped_publisher.py x y z roll pitch yaw from_frame rate"

def eulerToQuaternion(roll, pitch, yaw):
    "convert roll-pitch-yaw to quaternion"
    return quaternion_from_euler(roll, pitch, yaw)
    
if __name__ == "__main__":
    rospy.init_node("pose_stamped_publisher")
    argv = rospy.myargv()
    if len(argv) != 9:
        usage()
        sys.exit(1)
    x = float(argv[1])
    y = float(argv[2])
    z = float(argv[3])
    roll = float(argv[4])
    pitch = float(argv[5])
    yaw = float(argv[6])
    frame = argv[7]
    rate = float(argv[8])
    pose = PoseStamped()
    
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # roll, pitch, yaw to quaternion
    q = eulerToQuaternion(roll, pitch, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.x = q[2]
    pose.pose.orientation.w = q[3]
    r = rospy.Rate(rate)
    pub = rospy.Publisher("~output", PoseStamped)
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        pub.publish(pose)
        r.sleep()
