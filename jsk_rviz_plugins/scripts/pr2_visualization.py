#!/usr/bin/env python

import rospy

rospy.init_node("pr2_rviz_visualization")


from std_msgs.msg import Float32
from pr2_msgs.msg import BatteryServer

battery_status = {}
battery_pub0 = rospy.Publisher("/visualization/battery/value0", Float32)
battery_pub1 = rospy.Publisher("/visualization/battery/value1", Float32)
battery_pub2 = rospy.Publisher("/visualization/battery/value2", Float32)
battery_pub3 = rospy.Publisher("/visualization/battery/value3", Float32)
battery_pubs = [battery_pub0,
                battery_pub1,
                battery_pub2,
                battery_pub3]

def batteryCB(msg):
    battery_pubs[msg.id].publish(Float32(msg.averageCharge))

s = rospy.Subscriber("/battery/server", BatteryServer, batteryCB)

rospy.spin()


