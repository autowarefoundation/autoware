#!/usr/bin/env python

# Copyright (c) 2017, Analog Devices Inc.
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Ryosuke Tajima

import math
import unittest
import rospy
import rostest
import time
from sensor_msgs.msg import Imu

def imu_get_min(imu, a, b):
    imu.linear_acceleration.x = min(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = min(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = min(a.linear_acceleration.z, b.linear_acceleration.z)

def imu_get_max(imu, a, b):
    imu.linear_acceleration.x = max(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = max(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = max(a.linear_acceleration.z, b.linear_acceleration.z)

class TestImu(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_adxl345')

    def setUp(self):
        self.imu_raw_count = 0
        self.imu_count = 0
        self.imu_raw_min = Imu()
        self.imu_raw_max = Imu()
        self.imu_min = Imu()
        self.imu_max = Imu()
        rospy.Subscriber('/adxl345/data_raw', Imu, self.cb_imu_raw, queue_size=1000)
        rospy.sleep(0.1)
        
    def cb_imu_raw(self, msg):
        self.imu_raw_count += 1
        imu_get_min(self.imu_raw_min, self.imu_raw_min, msg)
        imu_get_max(self.imu_raw_max, self.imu_raw_max, msg)

    def test_imu_raw(self):
        self.assertTrue(self.imu_raw_count > 0, 'No data received from /adxl345/data_raw')
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.z));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.z));

if __name__ == '__main__':
    rostest.rosrun('adi_driver', 'test_adxl345', TestImu)
        
