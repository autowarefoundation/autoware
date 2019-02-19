#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from autoware_system_msgs.msg import SystemStatus

class SystemStatusSubscriber:
    def __init__(self):
        self.__functions=[]

    def enable(self):
        rate = rospy.Rate(1)
        self.__status_sub = rospy.Subscriber("/system_status", SystemStatus, self.systemStatusCallback)
        #while not rospy.is_shutdown():
        #    rate.sleep()
        rospy.spin()

    def systemStatusCallback(self,msg):
        for function_itr in self.__functions:
            function_itr(msg)

    def addCallback(self,func):
        self.__functions.append(func)
