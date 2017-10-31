#!/usr/bin/env python
# 
# Author: Jonathan Sprinkle
# Copyright (c) 2015-2016 Arizona Board of Regents
# All rights reserved.
# 
# Permission is hereby granted, without written agreement and without 
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the 
# above copyright notice and the following two paragraphs appear in 
# all copies of this software.
# 
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
# IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
# SUCH DAMAGE.
# 
# THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
# IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
# TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# This node will publish the path in which you've traveled. The publishing
# happens at 10Hz, unless no connection to odometry exists.
# The published path is appended whenever the odometry differs by at least
# 1m in the L1 norm---i.e., x or y changes by 1m---from the last point in the
# published path.

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
import sys, getopt

class odom2path:

    def __init__(self,ns):
        self.ns = ns
        rospy.init_node('odom2path', anonymous=True)

        # set so that whenever we receive on the odom topic, 
        # the callback method is called
        rospy.Subscriber('odom'.format(ns), Odometry, self.callback)
        # setup the state data for the publisher
        self.pub_path = rospy.Publisher('path'.format(ns), Path, queue_size=10)
        # we want to publish immediately when we receive a new data point
        self.publishNow = True
        # initialize the path message and its header
        self.pathMsg = Path()
        self.pathMsg.header = Header()
        # initial values are not provided (set to None)
        self.x = None
        self.y = None

    # This method is called whenever we receive from the subscriber above
    def callback(self,data):
        # we always publish right away
        self.publishNow = True
        # increment the header's sequence
        self.pathMsg.header.seq += 1
        # using rospy.Time means automatically getting sim time as appropriate
        self.pathMsg.header.stamp = rospy.Time.now()
        # the odometry frame is set here
        # TODO: set a parameter for the odometry frame
        self.pathMsg.header.frame_id = '{0}/odom'.format(self.ns)

        # Note that we append a new pose to the path ONLY if the position
        # has moved more than 1m from its previous spot (L1 norm)
        if self.x == None or (abs(self.x - data.pose.pose.position.x) > 1 
                           or abs(self.y - data.pose.pose.position.y) > 1):
            pose = PoseStamped()

            # copy over the values individually
            pose.header.frame_id='{0}/odom'.format(self.ns)
            pose.header.seq= len(self.pathMsg.poses)+1
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = float(data.pose.pose.position.x)
            pose.pose.position.y = float(data.pose.pose.position.y)
            pose.pose.orientation.x = float(data.pose.pose.orientation.x)
            pose.pose.orientation.y = float(data.pose.pose.orientation.y)
            pose.pose.orientation.z = float(data.pose.pose.orientation.z)
            pose.pose.orientation.w = float(data.pose.pose.orientation.w)
        
            self.pathMsg.poses.append(pose)

            self.x = float(data.pose.pose.position.x)
            self.y = float(data.pose.pose.position.y)

    # refer to the publishNow var, which is set during the receipt of a new msg
    def publish(self):
        if self.publishNow:
            rospy.logdebug(rospy.get_caller_id() + " publishing new path with {0} elements.".format(len(self.pathMsg.poses)))
            self.pub_path.publish(self.pathMsg)
            # after we publish, we ensure to wait until a new odom point arrives
            self.publishNow = False

def usage():
    print('odom2path -n catvehicle')


def main(argv):
    # here we must acquire the ns from the cmd line, so that we can 
    # ensure that we use the right frame for the tf of the path msgs
    ns=''
    try:
        opts, args = getopt.getopt(argv, "hn:", ["help", "namespace="])
    except getopt.GetoptError:
        usage()
        exit.sys()  

    for o, a in opts:
        if o == "help":
            usage()
            exit.sys()
        elif o in ("-n","-namespace"):
            ns=a
        else:
            usage()
            exit.sys()

    node = odom2path(ns)
    rate = rospy.Rate(10) # run at 10Hz
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
    try:
        listener('catvehicle')
    except rospy.ROSInterruptException:
        pass


