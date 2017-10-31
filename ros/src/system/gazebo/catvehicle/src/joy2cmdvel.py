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



import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Joy
import sys, getopt

# requires the ros-indigo-joysticks

class joy2cmdvel:

    def __init__(self):
        rospy.init_node('joy2cmdvel', anonymous=True)

        self.ns = rospy.get_param("~namespace","catvehicle")
        self.velmax = rospy.get_param("~velmax",3)

        rospy.loginfo(rospy.get_caller_id() + " startup in namespace {0} with max velocity {1}".format(self.ns,self.velmax))


        rospy.Subscriber('/joy'.format(self.ns), Joy, self.callback)
        self.pub_cmdvel = rospy.Publisher('{0}/cmd_vel'.format(self.ns), Twist, queue_size=1)

        self.x = 0
        self.z = 0

    def callback(self,data):
#        rospy.loginfo(rospy.get_caller_id() + " heard linear=%lf, angular=%lf", data.axes[3], data.axes[0])
        self.x = data.axes[3]*self.velmax
        self.z = data.axes[0]

    def publish(self):
        msgTwist = Twist()
        msgTwist.linear.x = self.x
        msgTwist.angular.z = self.z
        self.pub_cmdvel.publish(msgTwist)
        
def usage():
    print('joy2cmdvel -n catvehicle')


def main(argv):

    node = joy2cmdvel()
    rate = rospy.Rate(100) # run at 100Hz
    while not rospy.is_shutdown():
        node.publish()
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv[1:])
    try:
        listener('catvehicle')
    except rospy.ROSInterruptException:
        pass


