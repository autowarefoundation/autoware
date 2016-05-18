#!/usr/bin/env python

FRAME_ID = "/map"

from jsk_footstep_msgs.msg import Footstep, FootstepArray

import rospy

def main():
    pub = rospy.Publisher("/footsteps", FootstepArray)
    r = rospy.Rate(3)
    ysize = 0
    zpos = 0
    while not rospy.is_shutdown():
        msg = FootstepArray()
        now = rospy.Time.now()
        msg.header.frame_id = FRAME_ID
        msg.header.stamp = now
        xpos = 0
        
        for i in range(20):
            footstep = Footstep()
            if i % 2 == 0:
                footstep.leg = Footstep.LEFT
                footstep.pose.position.y = 0.21
            else:
                footstep.leg = Footstep.RIGHT
                footstep.pose.position.y = -0.21
            footstep.pose.orientation.w = 1.0
            footstep.pose.position.x = xpos
            footstep.pose.position.z = zpos
            footstep.dimensions.x = 0.25
            footstep.dimensions.y = 0.15
            footstep.dimensions.z = 0.01
            footstep.footstep_group = i / 5
            msg.footsteps.append(footstep)
            xpos = xpos + 0.25
            zpos = zpos + 0.1
            ysize = ysize + 0.01
            if ysize > 0.15:
                ysize = 0.0
            if zpos > 0.5:
                zpos = 0.0
        pub.publish(msg)
        r.sleep()

if __name__ == "__main__":
    rospy.init_node("footstep_sample")
    main()
    
