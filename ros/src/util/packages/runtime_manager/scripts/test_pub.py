#!/usr/bin/env python
#
# Copyright 2015-2019 Autoware Foundation
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

import sys
import rospy
import std_msgs.msg

if __name__ == "__main__":
	s = sys.argv[1] if len(sys.argv) > 1 else 'hello'
	rospy.init_node('test_pub', anonymous=True)
	pub = rospy.Publisher('to_rtmgr', std_msgs.msg.String, queue_size=10)
	r = rospy.Rate(10)
	r.sleep()
        pub.publish(s)
	r.sleep()

# EOF
