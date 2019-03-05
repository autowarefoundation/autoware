#!/usr/bin/python
#
# Copyright 2018-2019 Autoware Foundation
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
#
# v1.0 Jacob Lambert 2018-05-30
#
# Copyright (c) 2012,
# Systems, Robotics and Vision Group
# University of the Balearican Islands
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Systems, Robotics and Vision Group, University of
#       the Balearican Islands nor the names of its contributors may be used to
#       endorse or promote products derived from this software without specific
#       prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


PKG = 'autoware_bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse

def rpl_msg_time_with_hdr(inbag,outbag):
  rospy.loginfo('Processing input bagfile  : %s', inbag)
  rospy.loginfo('Writing to output bagfile : %s', outbag)
  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    # This also replaces tf timestamps under the assumption
    # that all transforms in the message share the same timestamp
    if topic == "/tf" and msg.transforms:
      outbag.write(topic, msg, msg.transforms[0].header.stamp)
    else:
      outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
  rospy.loginfo('Closing output bagfile and exit...')
  outbag.close();

if __name__ == "__main__":
  rospy.init_node('rpl_msg_time_with_hdr')
  parser = argparse.ArgumentParser(
      description='Create a new bagfile from an existing one replacing the message time for the header time.')
  parser.add_argument('-o', metavar='OUTPUT_BAGFILE', required=True, help='output bagfile')
  parser.add_argument('-i', metavar='INPUT_BAGFILE', required=True, help='input bagfile')
  args = parser.parse_args()
  try:
    rpl_msg_time_with_hdr(args.i, args.o)
  except Exception, e:
    import traceback
    traceback.print_exc()
