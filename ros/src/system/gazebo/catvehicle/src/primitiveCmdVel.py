#!/usr/bin/env python
# 
# Author: Jonathan Sprinkle
# Copyright (c) 2015 Arizona Board of Regents
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

# This node generates cmd_vel inputs to the vehicle in order to make it move
# around. Use the arrow keys to make the vehicle turn its wheels, move forward,
# or backward. If you want to move forward (but turn left), first press the left
# arrow key, then press up. 

import rospy
from geometry_msgs.msg import Twist
import sys, getopt, curses

class primitiveCmdVel:

    def __init__(self,ns):
        self.ns = ns
        rospy.init_node('primitiveCmdVel', anonymous=True)
        self.pub_cmd_vel = rospy.Publisher('{0}/cmd_vel'.format(ns), Twist, queue_size=1)

        self.x = 0
        self.z = 0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.pub_cmd_vel.publish(msg)
        
def usage():
    print('primitiveCmdVel -n catvehicle')


def main(argv):
    ns='catvehicle'
    try:
        opts, args = getopt.getopt(argv, "hn:", ["help", "namespace="])
    except getopt.GetoptError:
        usage()
        exit.sys()
    
    stdscr = curses.initscr()
    node = primitiveCmdVel(ns)
    while not rospy.is_shutdown():

# import curses
#stdscr = curses.initscr()
#c = stdscr.getch()
#print 'you entered', chr(c)
#curses.endwin()

        ch = stdscr.getch()

#        ch = sys.stdin.read(1)
        node.x = 4
        node.z = 0
        # left arrow
        if ch == chr(37):
            node.z = 1
        # up arrow
        elif ch == chr(38):
            node.x = node.x
        # right arrow
        elif ch == chr(39):
            node.z = -1
        # down arrow
        elif ch == chr(40):
            node.x = -node.x
        else:
            curses.endwin()

        node.publish()

    curses.endwin()

if __name__ == '__main__':
    main(sys.argv[1:])


