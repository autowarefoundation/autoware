#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32
from threading import Lock
#from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface

g_lock = Lock()
g_msg = None

def callback(msg):
    global g_msg, g_lock
    with g_lock:
        g_msg = msg

def config_callback(config, level):
    global g_format
    g_format = config.format
    return config

def publish_text(event):
    global g_lock, g_msg, g_format
    with g_lock:
        if not g_msg:
            return
        text_interface.publish(g_format.format(g_msg.data))

def publish_text_multi(event):
    global g_lock, multi_topic_msgs, g_format
    with g_lock:
        if all([msg for topic, msg in multi_topic_msgs.items()]):
            text_interface.publish(g_format.format(sum([msg.data for topic, msg in multi_topic_msgs.items()])))
        
multi_topic_msgs = dict()
        
class MultiTopicCallback():
    def __init__(self, topic):
        self.topic = topic
        global multi_topic_msgs
        multi_topic_msgs[self.topic] = None
    def callback(self, msg):
        global multi_topic_msgs
        with g_lock:
            multi_topic_msgs[self.topic] = msg
        
if __name__ == "__main__":
    rospy.init_node("float32_to_overlay_text")
    text_interface = OverlayTextInterface("~text")
    multi_topics = rospy.get_param("~multi_topics", [])
    g_format = rospy.get_param("~format", "value: {0}")
    if multi_topics:
        subs = []
        for topic in multi_topics:
            callback = MultiTopicCallback(topic)
            subs.append(rospy.Subscriber(topic, Float32, callback.callback))
        rospy.Timer(rospy.Duration(0.1), publish_text_multi)
    else:
        sub = rospy.Subscriber("~input", Float32, callback)
        rospy.Timer(rospy.Duration(0.1), publish_text)
    rospy.spin()
