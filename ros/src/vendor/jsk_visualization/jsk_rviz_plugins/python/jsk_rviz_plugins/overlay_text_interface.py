#!/usr/bin/env python

from jsk_rviz_plugins.cfg import OverlayTextInterfaceConfig
from jsk_rviz_plugins.msg import OverlayText
from dynamic_reconfigure.server import Server
import rospy

class OverlayTextInterface():
    def __init__(self, topic):
        self.srv = Server(OverlayTextInterfaceConfig, self.callback)
        self.pub = rospy.Publisher(topic, OverlayText)
    def callback(self, config, level):
        self.config = config
        return config
    def publish(self, text):
        msg = OverlayText()
        msg.text = text
        msg.width = self.config.width
        msg.height = self.config.height
        msg.top = self.config.top
        msg.left = self.config.left
        msg.fg_color.a = self.config.fg_alpha
        msg.fg_color.r = self.config.fg_red
        msg.fg_color.g = self.config.fg_green
        msg.fg_color.b = self.config.fg_blue
        msg.bg_color.a = self.config.bg_alpha
        msg.bg_color.r = self.config.bg_red
        msg.bg_color.g = self.config.bg_green
        msg.bg_color.b = self.config.bg_blue
        msg.text_size = self.config.text_size
        self.pub.publish(msg)
