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
