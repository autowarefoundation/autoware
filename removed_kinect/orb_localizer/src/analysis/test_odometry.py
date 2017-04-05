#!/usr/bin/python

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import math
from mayavi.tools.camera import yaw


x = 0.0
y = 0.0
yaw = 0.0
lastTimestamp = -1.0


def odometer (twistMsg):
    global x, y, yaw, lastTimestamp
    curTime = twistMsg.header.stamp.to_sec()
    if lastTimestamp<0 :
        lastTimestamp = curTime
    dt = curTime - lastTimestamp
    rtwist = twistMsg.twist.twist

    vx = rtwist.linear.x * math.sin(yaw)
    vy = rtwist.linear.x * math.cos(yaw)
    x = x + vx*dt
    y = y + vy*dt
    
    yaw += dt * rtwist.angular.z
    yaw = normalizeAngle(yaw)
    lastTimestamp = curTime
    
    print (curTime, x, y, yaw)


def normalizeAngle (angle):
    return angle % (2*math.pi)



if __name__ == '__main__' :
    
    rospy.init_node ('Odometer', anonymous=True)
    rospy.Subscriber('motion', TwistWithCovarianceStamped, odometer)
    rospy.spin()
    print ('Done')
    pass
