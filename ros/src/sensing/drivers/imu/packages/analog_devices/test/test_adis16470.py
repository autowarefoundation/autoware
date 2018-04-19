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
from collections import deque
from sensor_msgs.msg import Imu

def imu_get_min(imu, a, b):
    imu.angular_velocity.x = min(a.angular_velocity.x, b.angular_velocity.x)
    imu.angular_velocity.y = min(a.angular_velocity.y, b.angular_velocity.y)
    imu.angular_velocity.z = min(a.angular_velocity.z, b.angular_velocity.z)
    imu.linear_acceleration.x = min(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = min(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = min(a.linear_acceleration.z, b.linear_acceleration.z)

def imu_get_max(imu, a, b):
    imu.angular_velocity.x = max(a.angular_velocity.x, b.angular_velocity.x)
    imu.angular_velocity.y = max(a.angular_velocity.y, b.angular_velocity.y)
    imu.angular_velocity.z = max(a.angular_velocity.z, b.angular_velocity.z)
    imu.linear_acceleration.x = max(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = max(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = max(a.linear_acceleration.z, b.linear_acceleration.z)

class TestImu(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_imu')

    def setUp(self):
        self.imu_raw_count = 0
        self.imu_count = 0
        self.imu_raw_data = deque(maxlen=100)
        self.imu_data = deque(maxlen=100)
        rospy.Subscriber('/imu/data_raw', Imu, self.cb_imu_raw, queue_size=1000)
        rospy.Subscriber('/imu/data', Imu, self.cb_imu, queue_size=1000)
        
    def cb_imu_raw(self, msg):
        self.imu_raw_count += 1
        self.imu_raw_data.append(msg)

    def cb_imu(self, msg):
        self.imu_count += 1
        self.imu_data.append(msg)

    def test_imu_raw(self):
        time.sleep(1.0)
        # Check data count
        self.assertTrue(self.imu_raw_count>0, 'No data received from /imu/data_raw')

        # Check orientation
        for imu in self.imu_raw_data:
            self.assertAlmostEqual(imu.orientation.x, 0)
            self.assertAlmostEqual(imu.orientation.y, 0)
            self.assertAlmostEqual(imu.orientation.z, 0)
            self.assertAlmostEqual(imu.orientation.w, 1)
            for cov in imu.orientation_covariance:
                self.assertAlmostEqual(cov, 0)

        # Check angular velocity
        for imu in self.imu_raw_data:
            self.assertTrue(abs(imu.angular_velocity.x) < 0.1);
            self.assertTrue(abs(imu.angular_velocity.y) < 0.1);
            self.assertTrue(abs(imu.angular_velocity.z) < 0.1);
            for cov in imu.angular_velocity_covariance:
                self.assertAlmostEqual(cov, 0)

        # Check linear_acceleration with gravity (CAUTION: test will fail in space)
        for imu in self.imu_raw_data:
            accl = math.sqrt(imu.linear_acceleration.x**2 + imu.linear_acceleration.y**2 + imu.linear_acceleration.z**2)
            self.assertTrue(accl > 8.0)
            self.assertTrue(accl < 11.0)
            
    def test_imu(self):
        time.sleep(1.0)
        # Check data count
        self.assertTrue(self.imu_count>0, 'No data received from /imu/data')

        # Check orientation
        for imu in self.imu_data:
            quat = math.sqrt(imu.orientation.x**2 + imu.orientation.y**2 + imu.orientation.z**2 + imu.orientation.w**2)
            self.assertAlmostEqual(quat, 1, delta=0.001)

        # Check angular velocity
        for imu in self.imu_data:
            self.assertTrue(abs(imu.angular_velocity.x) < 0.1);
            self.assertTrue(abs(imu.angular_velocity.y) < 0.1);
            self.assertTrue(abs(imu.angular_velocity.z) < 0.1);
            for cov in imu.angular_velocity_covariance:
                self.assertAlmostEqual(cov, 0)

        # Check linear_acceleration with gravity (CAUTION: test will fail in space)
        for imu in self.imu_data:
            accl = math.sqrt(imu.linear_acceleration.x**2 + imu.linear_acceleration.y**2 + imu.linear_acceleration.z**2)
            self.assertTrue(accl > 8.0)
            self.assertTrue(accl < 11.0)

if __name__ == '__main__':
    rostest.rosrun('adi_driver', 'test_adi_driver', TestImu)
        
