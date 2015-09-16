#!/usr/bin/env python
"""
  Copyright (c) 2015, Nagoya University
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


import os
import sys
import rospy
import yaml
import subprocess
from std_msgs.msg import String

str_fn_dic = {}

def load_yaml(filename, def_ret=None):
	dir = os.path.abspath(os.path.dirname(__file__)) + "/"
	path = dir + filename
	if not os.path.isfile(path):
		return def_ret
	print('loading ' + filename)
	f = open(dir + filename, 'r')
	d = yaml.load(f)
	f.close()
	return d

def node_is_exist(node_name):
	subprocess.call([ 'sh', '-c', 'echo y | rosnode cleanup' ])
	run_nodes = subprocess.check_output([ 'rosnode', 'list' ]).strip().split('\n')
	return node_name in run_nodes

def callback(data):
	fn = str_fn_dic.get(data.data)
	if fn is None:
		print('Invalid msg str "' + data.data +'"')
		return
	subprocess.call(['rosrun', 'sound_play', 'play.py', fn] )

def sound_player():
	proc = None
	if node_is_exist('/sound_play'):
		print('already, sound_play exist.')
	else:
		proc = subprocess.Popen(['rosrun', 'sound_play', 'soundplay_node.py'])

	pack_path = subprocess.check_output([ 'rospack', 'find', 'sound_player' ]).strip()

	global str_fn_dic
	str_fn_dic = load_yaml('sound_player.yaml', {})

	for (k,fn) in str_fn_dic.items():
		fn = os.path.expandvars(os.path.expanduser(fn))
		if not os.path.isabs(fn):
			if '/' in fn:
				fn = pack_path + '/' + fn
			else:
				cmd = 'find ' + pack_path + '/wavs' + ' -name ' + fn
				fn = subprocess.check_output([ 'sh', '-c', cmd ]).strip().split('\n')[0]
		str_fn_dic[k] = fn

	rospy.init_node('sound_player', anonymous=True)
	rospy.Subscriber('sound_player', String, callback);
	rospy.spin()

	if proc:
		proc.terminate()

if __name__ == '__main__':
	sound_player()

# EOF
