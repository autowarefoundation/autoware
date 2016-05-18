#!/usr/bin/env python

from hrpsys_ros_bridge.msg import MotorStates
from jsk_rviz_plugins.msg import OverlayText
from sensor_msgs.msg import JointState
#from sensor_msgs.msg import JointState as MotorStates
from std_msgs.msg import Float32
import rospy
from urdf_parser_py.urdf import URDF


g_publishers = []
g_effort_publishers = {}
g_text_publisher = None
safe_color = (52, 152, 219)     # ~50
warning_color = (230, 126, 34)               # 50~
fatal_color = (231, 76, 60)                   # 70~
def allocatePublishers(num):
    global g_publishers
    if num > len(g_publishers):
        for i in range(len(g_publishers), num):
            pub = rospy.Publisher('temperature_%02d' % (i), Float32)
            g_publishers.append(pub)
def allocateEffortPublishers(names):
    global g_effort_publishers
    for name in names:
        if not g_effort_publishers.has_key(name):
            g_effort_publishers[name] = rospy.Publisher('effort_%s' % (name), Float32)
        
def motorStatesCallback(msg):
    global g_publishers, g_text_publisher
    #values = msg.position
    values = msg.temperature
    names = msg.name
    allocatePublishers(len(values))
    max_temparature = 0
    max_temparature_name = ""
    for name, val, pub in zip(names, values, g_publishers):
        pub.publish(Float32(val))
        if max_temparature < val:
            max_temparature = val
            max_temparature_name = name
    if max_temparature:
        if max_temparature > 70:
            color = fatal_color
        elif max_temparature > 50:
            color = warning_color
        else:
            color = safe_color
        text = OverlayText()
        text.fg_color.r = color[0] / 255.0
        text.fg_color.g = color[1] / 255.0
        text.fg_color.b = color[2] / 255.0
        text.fg_color.a = 1.0
        text.bg_color.a = 0.0
        text.text = "%dC -- (%s)"  % (int(max_temparature), max_temparature_name)
        g_text_publisher.publish(text)

def jointStatesCallback(msg):
    global g_effort_publishers, robot_model
    values = msg.effort
    names = msg.name
    allocateEffortPublishers(names)
    for val, name in zip(values, names):
        # lookup effort limit
        candidate_joints = [j for j in robot_model.joints if j.name == name]
        if candidate_joints:
            limit = candidate_joints[0].limit.effort
            if limit == 0:
                rospy.logwarn("%s effort limit is 0" % (name))
            else:
                g_effort_publishers[name].publish(Float32(abs(val/limit)))

if __name__ == "__main__":
    rospy.init_node("motor_state_temperature_decomposer")
    robot_model = URDF.from_xml_string(rospy.get_param("/robot_description"))
    g_text_publisher = rospy.Publisher("max_temparature_text", OverlayText)
    s = rospy.Subscriber("/motor_states", MotorStates, motorStatesCallback)
    s_joint_states = rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
    #s = rospy.Subscriber("joint_states", MotorStates, motorStatesCallback)
    rospy.spin()
    
