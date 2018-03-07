#!/usr/bin/env python

# Copyright 2017 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
from prius_msgs.msg import Control
from sensor_msgs.msg import Joy


STEERING_AXIS = 0
THROTTLE_AXIS = 4


class Translator:
    def __init__(self):
        self.sub = rospy.Subscriber("joy", Joy, self.callback)
        self.pub = rospy.Publisher('prius', Control, queue_size=1)
        self.last_published_time = rospy.get_rostime()
        self.last_published = None
        self.timer = rospy.Timer(rospy.Duration(1./20.), self.timer_callback)
        
    def timer_callback(self, event):
        if self.last_published and self.last_published_time < rospy.get_rostime() + rospy.Duration(1.0/20.):
            self.callback(self.last_published)

    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message.axes)
        command = Control()
        command.header = message.header
        if message.axes[THROTTLE_AXIS] >= 0:
            command.throttle = message.axes[THROTTLE_AXIS]
            command.brake = 0.0
        else:
            command.brake = message.axes[THROTTLE_AXIS] * -1
            command.throttle = 0.0

        if message.buttons[3]:
            command.shift_gears = Control.FORWARD
        elif message.buttons[1]:
            command.shift_gears = Control.NEUTRAL
        elif message.buttons[0]:
            command.shift_gears = Control.REVERSE
        else:
            command.shift_gears = Control.NO_COMMAND

        command.steer = message.axes[STEERING_AXIS]
        self.last_published = message
        self.pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('joy_translator')
    t = Translator()
    rospy.spin()
