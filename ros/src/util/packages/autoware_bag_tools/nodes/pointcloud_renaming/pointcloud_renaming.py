#!/usr/bin/env python
"""
  Copyright (c) 2018, Nagoya University
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the name of Autoware nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

__author__ = "Alexander Carballo"
__email__ = "alexander@g.sp.m.is.nagoya-u.ac.jp"
__copyright__ = "Copyright (c) 2018, Nagoya University"
__version__ = "1.0"
__date__ = "2018/12/04"

"""
Allows to rename the topic name and frame name of a pointcloud topic inside a BAG file.
Additionally, makes sure the field name for the intensity channel is actually 'intensity'.
"""

import sys
import rosbag
import argparse
import progressbar

def main():
    
    parser = argparse.ArgumentParser(description = "Rename intensity field on rosbag")
    parser.add_argument("-i", "--input", required=True, help = "Input BAG filename.")
    parser.add_argument("-o", "--output", default = "output.bag", help = "Output BAG filename.")
    parser.add_argument("-t", "--topic", default = "/kitti/velo/pointcloud", help = "PointCloud topic name to change (empty to change all pointcloud topics).")
    parser.add_argument("-n", "--newtopic", default = "", help = "New topic name to use instead of the original (empty to avoid changing the name).")
    parser.add_argument("-f", "--frame", default = "", help = "New frame name to use instead of the original (empty to avoid changing the name).")
    args = parser.parse_args()
    
    if args.input == None:
        print("Input BAG filename not given.")
        parser.print_help()
        sys.exit(1)
    
    with rosbag.Bag(args.output, 'w') as outbag:
    	inbag = rosbag.Bag(args.input, 'r')
    	instances = inbag.get_message_count()
    	progress = progressbar.ProgressBar(maxval=instances)
    	progress.start()
    	instance = 0
        for topic, msg, t in inbag.read_messages():
            # This replaces the pointcloud 4th field name to "intensity"
            # Additionally, topic name and frame name can be replaced
            
            #if msg type is sensor_msgs/PointCloud2 and arg.topic either matches the current topic or is empty (any name)
            if msg._type == 'sensor_msgs/PointCloud2' and (topic == args.topic or args.topic == ''):
            	new_topic = topic
            	if args.topic <> '' and args.newtopic <> '':
                    new_topic = args.newtopic
                if args.frame <> '':
            	    msg.header.frame_id = args.frame
            	if len(msg.fields) == 4: #Assuming X,Y,Z,I format
            	    msg.fields[3].name = "intensity"
                outbag.write(new_topic, msg, msg.header.stamp if msg._has_header else t)
            else: #Any other topics are kept
                outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
            
            progress.update(instance)
            instance += 1
        
        
        progress.finish()
    
if __name__ == '__main__':
    main()
