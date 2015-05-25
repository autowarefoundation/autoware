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


import wx
import wx.lib.buttons
import wx.lib.agw.customtreectrl as CT
import gettext
import os
import re
import sys
import socket
import struct
import shlex
import signal
import subprocess
import psutil
import yaml
import datetime
import rtmgr
import rospy
import std_msgs.msg
from decimal import Decimal
from runtime_manager.msg import ConfigCarDpm
from runtime_manager.msg import ConfigPedestrianDpm
from runtime_manager.msg import ConfigNdt
from runtime_manager.msg import ConfigNdtSlam
from runtime_manager.msg import ConfigNdtSlamOutput
from runtime_manager.msg import ConfigLaneFollower
from runtime_manager.msg import ConfigCarKf
from runtime_manager.msg import ConfigPedestrianKf
from runtime_manager.msg import ConfigLaneRule
from runtime_manager.msg import ConfigWaypointLoader
from runtime_manager.msg import ConfigCarFusion
from runtime_manager.msg import ConfigPedestrianFusion
from ui_socket.msg import mode_cmd
from ui_socket.msg import gear_cmd
from ui_socket.msg import Waypoint
from ui_socket.msg import route_cmd
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from runtime_manager.msg import accel_cmd
from runtime_manager.msg import steer_cmd
from runtime_manager.msg import brake_cmd
from runtime_manager.msg import traffic_light

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)
		self.all_procs = []
		self.all_procs_nodes = {}
		self.all_cmd_dics = []
		self.load_dic = self.load_yaml('param.yaml', def_ret={})
		self.config_dic = {}
		self.selector = {}
		self.Bind(wx.EVT_CLOSE, self.OnClose)

		#
		# ros
		#
		rospy.init_node('runime_manager', anonymous=True)
		rospy.Subscriber('to_rtmgr', std_msgs.msg.String, self.RosCb)
		self.pub = rospy.Publisher('from_rtmgr', std_msgs.msg.String, queue_size=10)

		#
		# for Main tab
		#
		tab = self.notebook_1_pane_1

		for nm in [ 'tablet', 'mobile', 'vehicle', 'database' ]:
			setattr(self, 'bitmap_' + nm, self.get_static_bitmap(tab, nm+'.png', 0.3))

		self.main_cmd = {}
		self.all_cmd_dics.append(self.main_cmd)
		self.main_dic = self.load_yaml('main.yaml')

		self.params = []
		self.add_params(self.main_dic.get('params', []))
		self.selector.update(self.main_dic.get('selector', {}))

		self.setup_buttons(self.main_dic.get('buttons', {}), self.main_cmd)

		self.route_cmd_waypoint = [ Waypoint(0,0), Waypoint(0,0) ]
		rospy.Subscriber('route_cmd', route_cmd, self.route_cmd_callback)

		for k in [ 'gnss', 'pmap', 'vmap', 'ndt', 'lf' ]:
			name = k + '_stat'
			setattr(self, name, False)
			rospy.Subscriber(name, std_msgs.msg.Bool, getattr(self, name + '_callback', None))
		self.map_stat = False
		self.bak_main_button_color = self.button_sensor.GetForegroundColour()

		szr = wx.BoxSizer(wx.VERTICAL)
		for cc in self.main_dic.get('control_check', []):
			pdic = {}
			prm = self.get_param(cc.get('param'))
			for var in prm['vars']:
				pdic[ var['name'] ] = var['v']
			gdic = self.gdic_get_1st(cc)
			panel = ParamPanel(self.panel_main_cc, frame=self, pdic=pdic, gdic=gdic, prm=prm)
			szr.Add(panel, 0, wx.EXPAND)
		self.panel_main_cc.SetSizer(szr)

		#
		# for Computing tab
		#
		parent = self.tree_ctrl_0.GetParent()
		for i in range(3):
			self.obj_get('tree_ctrl_' + str(i)).Destroy()
		items = self.load_yaml('computing_launch_cmd.yaml')

		self.add_params(items.get('params', []))

		self.computing_cmd = {}
		self.all_cmd_dics.append(self.computing_cmd)
		for i in range(3):
			tree_ctrl = self.create_tree(parent, items['subs'][i], None, None, self.computing_cmd)
			tree_ctrl.ExpandAll()
			tree_ctrl.SetHyperTextVisitedColour(tree_ctrl.GetHyperTextNewColour()) # no change
			setattr(self, 'tree_ctrl_' + str(i), tree_ctrl)

		self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)
		self.Bind(CT.EVT_TREE_ITEM_HYPERLINK, self.OnTreeHyperlinked)

		#
		# for Sensing Tab
		#
		self.drv_probe_cmd = {}
		self.sensing_cmd = {}
		self.all_cmd_dics.append(self.sensing_cmd)
		dic = self.load_yaml('sensing.yaml')

		self.add_params(dic.get('params', []))
		self.selector.update(dic.get('selector', {}))

		self.create_checkboxes(dic, self.panel_sensing, None, self.drv_probe_cmd, self.sensing_cmd, self.OnSensingDriver)

		if 'buttons' in dic:
			self.setup_buttons(dic['buttons'], self.sensing_cmd)

		self.set_param_panel(self.button_points_image, self.panel_points_image)
		self.set_param_panel(self.button_scan_image, self.panel_scan_image)
		self.set_param_panel(self.button_virtual_scan_image, self.panel_virtual_scan_image)

		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.OnProbe, self.timer)
		self.probe_interval = 10*1000
		if self.checkbox_auto_probe.GetValue():
			self.OnProbe(None)
			self.timer.Start(self.probe_interval)

		self.dlg_rosbag_record = MyDialogRosbagRecord(self, cmd_dic=self.sensing_cmd)

		#
		# for Simulation Tab
		#
		self.simulation_cmd = {}
		self.all_cmd_dics.append(self.simulation_cmd)
		dic = self.load_yaml('simulation_launch_cmd.yaml')

		self.add_params(dic.get('params', []))
		self.selector.update(dic.get('selector', {}))

		self.create_checkboxes(dic, self.panel_simulation, None, None, self.simulation_cmd, self.OnSimulation)

		if 'buttons' in dic:
			self.setup_buttons(dic['buttons'], self.simulation_cmd)
		if 'checkboxs' in dic:
			self.setup_buttons(dic['checkboxs'], self.simulation_cmd)

		self.set_param_panel(self.button_launch_vmap, self.panel_vmap_prm)
		self.set_param_panel(self.button_launch_trajectory, self.panel_trajectory_prm)

		try:
			cmd = ['rosparam', 'get', '/use_sim_time']
			if subprocess.check_output(cmd, stderr=open(os.devnull, 'wb')).strip() == 'true':
				self.checkbox_sim_time.SetValue(True)
		except subprocess.CalledProcessError:
			pass

		#
		# for Data Tab
		#
		self.data_cmd = {}
		self.all_cmd_dics.append(self.data_cmd)
		dic = self.load_yaml('data.yaml')

		self.add_params(dic.get('params', []))

		parent = self.tree_ctrl_data.GetParent()
		self.tree_ctrl_data.Destroy()
		tree_ctrl = self.create_tree(parent, dic, None, None, self.data_cmd)
		tree_ctrl.ExpandAll()
		tree_ctrl.SetHyperTextVisitedColour(tree_ctrl.GetHyperTextNewColour()) # no change
		self.tree_ctrl_data = tree_ctrl

		self.setup_config_param_pdic()

		if 'buttons' in dic:
			self.setup_buttons(dic['buttons'], self.data_cmd)

		rtmgr.MyFrame.__do_layout(self)

		#
		# for Viewer Tab
		#
		self.viewer_cmd = {}
		self.all_cmd_dics.append(self.viewer_cmd)
		parent = self.panel_viewer
		sizer = self.sizer_viewer
		lst = self.load_yaml('viewer.yaml', {}).get('viewers', [])
		self.create_viewer_btns(parent, sizer, lst)

		self.nodes_dic = self.nodes_dic_get()

		#
		# for All
		#
		self.alias_grps = [
			[ self.button_launch_rosbag_play, self.button_launch_main_rosbag_play, ],
			[ self.button_kill_rosbag_play, self.button_kill_main_rosbag_play, ],
			[ self.button_pause_rosbag_play, self.button_pause_main_rosbag_play, ],
			[ self.text_ctrl_file_rosbag_play, self.text_ctrl_main_rosbag_play, ],
			[ self.button_ref_file_rosbag_play, self.button_ref_main_rosbag_play, ],
			[ self.text_ctrl_rate_rosbag_play, self.text_ctrl_rate_main_rosbag_play, ],
			[ self.checkbox_clock_rosbag_play, self.checkbox_clock_main_rosbag_play, ],
			[ self.checkbox_sim_time, self.checkbox_main_sim_time, ]
		]
		for grp in self.alias_grps:
			wx.CallAfter(self.alias_sync, get_top(grp))

		# for init button
		paths = [ os.environ['HOME'] + '/.autoware/data/tf',
			  os.environ['HOME'] + '/.autoware/data/map/pointcloud_map',
			  os.environ['HOME'] + '/.autoware/data/map/vector_map' ]
		for path in paths:
			if not os.path.exists(path):
				subprocess.call([ 'mkdir', '-p', path ])

	def __do_layout(self):
		pass

	def OnClose(self, event):
		self.kill_all()

		save_dic = {}
		for (name, pdic) in self.load_dic.items():
			if pdic and pdic != {}:
				save_dic[name] = pdic
		if save_dic != {}:
			dir = os.path.abspath(os.path.dirname(__file__)) + "/"
			print('saving param.yaml')
			f = open(dir + 'param.yaml', 'w')
			s = yaml.dump(save_dic, default_flow_style=False)
			#print 'save\n', s # for debug
			f.write(s)
			f.close()

		shutdown_sh = self.get_autoware_dir() + '/ros/shutdown'
		if os.path.exists(shutdown_sh):
			os.system(shutdown_sh)

		self.Destroy()

	def RosCb(self, data):
		print('recv topic msg : ' + data.data)

		r = rospy.Rate(10)
		rospy.is_shutdown()
		r.sleep()
		self.pub.publish(data.data)
		r.sleep()

	def setup_buttons(self, d, run_dic):
		for (k,d2) in d.items():
			obj = get_top( self.key_objs_get([ 'button_', 'button_launch_', 'checkbox_' ], k) )
			if not obj:
				s = 'button_launch_' + k
				setattr(self, s, s)
				obj = s
				s = 'button_kill_' + k
				setattr(self, s, s)
			if not d2 or type(d2) is not dict:
				continue
			if 'run' in d2:
				run_dic[obj] = (d2['run'], None)
			if 'param' in d2:
				pdic = self.load_dic.get(k, {})
				self.load_dic[k] = pdic
				prm = self.get_param(d2.get('param'))
				for var in prm.get('vars'):
					name = var.get('name')
					if name not in pdic and 'v' in var:
						pdic[name] = var.get('v')
				gdic = self.gdic_get_1st(d2)

				for (name, v) in pdic.items():
					restore = eval( gdic.get(name, {}).get('restore', 'lambda a : None') )
					restore(v)

				self.add_cfg_info(obj, obj, k, pdic, gdic, False, prm)

	#
	# Main Tab
	#
	def OnMainButton(self, event):
		obj = event.GetEventObject()
		self.OnLaunchKill_obj(obj)

	def OnDrive(self, event):
		obj = event.GetEventObject()
		v = obj.GetValue()
		pub = rospy.Publisher('mode_cmd', mode_cmd, queue_size=10)
		pub.publish(mode_cmd(mode=v))

	def OnClear(self, event):
		self.kill_all()

	def OnNetConn(self, event):
		obj = event.GetEventObject()
		self.launch_kill_proc(obj, self.main_cmd)

	#def OnReadNavi(self, event):
	#	self.text_ctrl_route_from_lat.SetValue(str(self.route_cmd_waypoint[0].lat))
	#	self.text_ctrl_route_from_lon.SetValue(str(self.route_cmd_waypoint[0].lon))
	#	self.text_ctrl_route_to_lat.SetValue(str(self.route_cmd_waypoint[1].lat))
	#	self.text_ctrl_route_to_lon.SetValue(str(self.route_cmd_waypoint[1].lon))

	def OnTextRoute(self, event):
		pass

	def OnLoadMap(self, event):
		obj = event.GetEventObject()
		self.OnSelector_obj(obj)

	def OnGear(self, event):
		grp = { self.button_statchk_d : 1,
			self.button_statchk_r : 2,
			self.button_statchk_b : 3,
			self.button_statchk_n : 4 }
		self.radio_action(event, grp.keys())
		v = grp.get(event.GetEventObject())
		if v is not None:
			pub = rospy.Publisher('gear_cmd', gear_cmd, queue_size=10)
			pub.publish(gear_cmd(gear=v))

	def OnProgManu(self, event):
		grp = { self.button_statchk_prog : 1,
			self.button_statchk_manu : 0 }
		self.radio_action(event, grp.keys())
		v = grp.get(event.GetEventObject())
		if v is not None:
			pub = rospy.Publisher('mode_cmd', mode_cmd, queue_size=10)
			pub.publish(mode_cmd(mode=v))

	def radio_action(self, event, grp):
		push = event.GetEventObject()
		for b in grp:
			v = b.GetValue()
			act = None
			act = True if b is push and not v else act
			act = False if b is not push and v else act
			if act is not None:
				b.SetValue(act)

	def stat_label_off(self, obj):
		(_, gdic, _) = self.obj_to_pdic_gdic_prm(obj)
		if gdic is None:
			gdic = {}
		data = std_msgs.msg.Bool(False)
		for k in gdic.get('stat_topic', []):
			cb = getattr(self, k + '_stat_callback', None)
			if cb:
				cb(data)

	def route_cmd_callback(self, data):
		self.route_cmd_waypoint = data.point

	def gnss_stat_callback(self, data):
		self.stat_set('gnss', data.data)
		self.main_button_update(self.button_perception, self.gnss_stat and self.ndt_stat)

	def pmap_stat_callback(self, data):
		self.pmap_stat = data.data
		self.stat_set('map', self.pmap_stat and self.vmap_stat)
		self.main_button_update(self.button_map, self.map_stat)

	def vmap_stat_callback(self, data):
		self.vmap_stat = data.data
		self.stat_set('map', self.pmap_stat and self.vmap_stat)
		self.main_button_update(self.button_map, self.map_stat)

	def ndt_stat_callback(self, data):
		self.stat_set('ndt', data.data)
		self.main_button_update(self.button_perception, self.gnss_stat and self.ndt_stat)

	def lf_stat_callback(self, data):
		self.stat_set('lf', data.data)
		self.main_button_update(self.button_control, self.lf_stat)

	def stat_set(self, k, stat):
		name = k + '_stat'
		setattr(self, name, stat)
		lb = getattr(self, 'label_' + name, None)
		if lb:
			wx.CallAfter(lb.Enable, stat)

	def main_button_update(self, obj, ready):
		wx.CallAfter(obj.SetForegroundColour, 'blue' if ready else self.bak_main_button_color)

	#
	# Computing Tab
	#
	def OnTreeChecked(self, event):
		self.OnChecked_obj(event.GetItem())

	def OnChecked_obj(self, obj):
		self.OnLaunchKill_obj(obj)

	def OnTreeHyperlinked(self, event):
		self.OnHyperlinked_obj(event.GetItem())

	def OnHyperlinked_obj(self, obj):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		if pdic is None or prm is None:
			return
		klass_dlg = globals().get(gdic.get('dialog', 'MyDialogParam'), MyDialogParam)
		dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
		dlg.ShowModal()

	def obj_to_add_args(self, obj):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		if pdic is None or prm is None:
			return None
		self.update_func(pdic, gdic, prm)
		s = ''
		for var in prm.get('vars'):
			cmd_param = var.get('cmd_param')
			if cmd_param:
				name = var.get('name')
				v = pdic.get(name)
				if (v is None or v == '') and 'default' in cmd_param:
					v = cmd_param.get('default')					
				if cmd_param.get('must') and (v is None or v == ''):
					print 'cmd_param', name, 'is must'
					wx.MessageBox('cmd_param ' + name + ' is must')
					return False
				if cmd_param.get('only_enable') and not v:
					continue
				name = cmd_param.get('var_name', name)
				unpack = cmd_param.get('unpack')
				if unpack is not None:
					v = ' '.join( v.split(unpack) )
				add = ''
				dash = cmd_param.get('dash')
				if dash is not None:
					add += dash + name
				delim = cmd_param.get('delim')
				if delim is not None:
					str_v = str(v)
					if var.get('kind') is None:
						str_v = adjust_num_str(str_v)
					if var.get('kind') == 'path':
						str_v = os.path.expandvars(os.path.expanduser(str_v))
					add += delim + str_v
				if add != '':
					s += add + ' '
		return s.strip(' ').split(' ') if s != '' else None

	def obj_to_pdic_gdic_prm(self, obj):
		info = self.config_dic.get(obj)
		if info is None:
			info = get_top([ v for v in self.config_dic.values() if v.get('obj') is obj ])
			if info is None:
				return (None, None, None)
		pdic = info.get('pdic')
		prm = info.get('param')
		gdic = info.get('gdic')
		return (pdic, gdic, prm)

	def update_func(self, pdic, gdic, prm):
		for var in prm.get('vars', []):
			name = var.get('name')
			gdic_v = gdic.get(name, {})
			func = gdic_v.get('func')
			if func is None:
				continue
			v = eval(func) if type(func) is str else func()
			pdic[ name ] = v
		if 'pub' in prm:
			self.publish_param_topic(pdic, prm)
		self.rosparam_set(pdic, prm)
		self.update_depend_enable(pdic, gdic, prm)

	def update_depend_enable(self, pdic, gdic, prm):
		for var in prm.get('vars', []):
			name = var.get('name')
			gdic_v = gdic.get(name, {})
			depend = gdic_v.get('depend')
			if depend is None:
				continue
			vp = gdic_v.get('var')
			if vp is None:
				continue
			v = pdic.get(depend)
			if v is None:
				continue
			depend_bool = eval( gdic_v.get('depend_bool', 'lambda v : bool(v)') )
			v = depend_bool(v)
			if vp.IsEnabled() != v:
				vp.Enable(v)

	def publish_param_topic(self, pdic, prm):
		pub = prm['pub']
		klass_msg = globals()[ prm['msg'] ]
		msg = klass_msg()

		for (name, v) in pdic.items():
			if prm.get('topic') == '/twist_cmd' and name == 'twist.angular.z':
				v = -v
			(obj, attr) = msg_path_to_obj_attr(msg, name)
			if obj and attr in obj.__slots__:
				type_str = obj._slot_types[ obj.__slots__.index(attr) ]
				setattr(obj, attr, str_to_rosval(v, type_str, v))
		
		if 'stamp' in prm.get('flags', []):
			(obj, attr) = msg_path_to_obj_attr(msg, 'header.stamp')
			setattr(obj, attr, rospy.get_rostime())

		pub.publish(msg)

	def rosparam_set(self, pdic, prm):
		cmd = [ 'rosparam', 'list' ]
		rosparams = subprocess.check_output(cmd).strip().split('\n')
		for var in prm.get('vars', []):
			name = var['name']
			if 'rosparam' not in var or name not in pdic:
				continue
			rosparam = var['rosparam']
			v = pdic.get(name)
			v = str(v)
			exist = rosparam in rosparams
			if exist:
				cmd = [ 'rosparam', 'get', rosparam ]
				ov = subprocess.check_output(cmd).strip()
				if ov == v:
					continue
			elif v == '':
				continue
			cmd = [ 'rosparam', 'set', rosparam, v ] if v != '' else [ 'rosparam', 'delete', rosparam ]
			print(cmd)
			subprocess.call(cmd)

	def OnRefresh(self, event):
		subprocess.call([ 'sh', '-c', 'echo y | rosnode cleanup' ])
		run_nodes = subprocess.check_output([ 'rosnode', 'list' ]).strip().split('\n')
		run_nodes_set = set(run_nodes)
		targ_objs = self.computing_cmd.keys() + self.data_cmd.keys()
		for (obj, nodes) in self.nodes_dic.items():
			if obj not in targ_objs:
				continue
			if getattr(obj, 'GetValue', None) is None:
				continue
			if nodes is None or nodes == []:
				continue
			#v = nodes and run_nodes_set.issuperset(nodes)
			v = len(run_nodes_set.intersection(nodes)) != 0
			if obj.GetValue() != v:
				if obj.IsEnabled() and not v:
					self.kill_obj(obj)
				set_val(obj, v)
				obj.Enable(not v)

	#
	# Viewer Tab
	#
	def create_viewer_btns(self, parent, sizer, lst):
		for dic in lst:
			lb = dic.get('label')
			prop = 0
			flag = wx.ALL | wx.EXPAND
			border = 4
			if 'subs' in dic:
				if lb:
					obj = static_box_sizer(parent, lb)
				else:
					obj = wx.BoxSizer(wx.VERTICAL)
				self.create_viewer_btns(parent, obj, dic['subs'])
			else:
				obj = wx.ToggleButton(parent, wx.ID_ANY, lb)
				self.Bind(wx.EVT_TOGGLEBUTTON, self.OnViewer, obj)
				self.viewer_cmd[obj] = (dic['cmd'], None)

			if sizer is self.sizer_viewer:
				prop = 1
				flag = wx.ALL | wx.ALIGN_CENTER_VERTICAL
			sizer.Add(obj, prop, flag, border)

	def OnViewer(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.viewer_cmd)

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.OnChecked_obj(event.GetEventObject())

	def OnAutoProbe(self, event):
		if event.GetEventObject().GetValue():
			self.OnProbe(None)
			self.timer.Start(self.probe_interval)
		else:
			self.timer.Stop()

	def OnProbe(self, event):
		#print('probe') # for debug
		items = self.drv_probe_cmd.items()
		for (obj, (cmd, bak_res)) in items:
			res = (os.system(cmd) == 0) if cmd else False
			if res == bak_res:
				continue
			self.drv_probe_cmd[obj] = (cmd, res)
			cfg_obj = self.get_cfg_obj(obj)
			en = obj.IsShown()
			if res and not en:
				obj.Show()
				if cfg_obj:
					cfg_obj.Show()
				continue
			if not res and en:
				v = obj.GetValue()
				if v:
					obj.SetValue(False)	
					self.launch_kill_proc(obj, self.sensing_cmd)
				obj.Hide()
				if cfg_obj:
					cfg_obj.Hide()

	def OnRosbagRecord(self, event):
		self.dlg_rosbag_record.Show()

	def create_checkboxes(self, dic, panel, sizer, probe_dic, run_dic, bind_handler):
		if 'name' not in dic:
			return
		obj = None
		bdr_flg = wx.ALL
		if 'subs' in dic:
			if dic['name']:
				obj = static_box_sizer(panel, dic.get('name'))
			else:
				obj = wx.BoxSizer(wx.VERTICAL)
			for d in dic['subs']:
				self.create_checkboxes(d, panel, obj, probe_dic, run_dic, bind_handler)
		else:
			obj = wx.CheckBox(panel, wx.ID_ANY, dic['name'])
			self.Bind(wx.EVT_CHECKBOX, bind_handler, obj)
			bdr_flg = wx.LEFT | wx.RIGHT
			if 'probe' in dic:
				probe_dic[obj] = (dic['probe'], None)
			if 'run' in dic:
				run_dic[obj] = (dic['run'], None)
			if 'param' in dic:
				obj = self.add_config_link(dic, panel, obj)
		if sizer:
			sizer.Add(obj, 0, wx.EXPAND | bdr_flg, 4)
		else:
			panel.SetSizer(obj)

	def add_config_link(self, dic, panel, obj):
		cfg_obj = wx.HyperlinkCtrl(panel, wx.ID_ANY, '[config]', '')
		cfg_obj.SetVisitedColour(cfg_obj.GetNormalColour()) # no change
		self.Bind(wx.EVT_HYPERLINK, self.OnConfig, cfg_obj)
		hszr = wx.BoxSizer(wx.HORIZONTAL)
		hszr.Add(obj)
		hszr.Add(wx.StaticText(panel, wx.ID_ANY, '  '))
		hszr.Add(cfg_obj)
		name = dic['name']
		pdic = self.load_dic.get(name, {})
		self.load_dic[name] = pdic
		gdic = self.gdic_get_1st(dic)
		prm = self.get_param(dic.get('param'))
		self.add_cfg_info(cfg_obj, obj, name, pdic, gdic, True, prm)
		return hszr

	#
	# Simulation Tab
	#
	def OnSimulation(self, event):
		self.OnChecked_obj(event.GetEventObject())

	def OnSimTime(self, event):
		obj = event.GetEventObject()
		self.alias_sync(obj)
		obj = self.alias_grp_top_obj(obj)
		cmd_dic = self.simulation_cmd
		(cmd, proc) = cmd_dic.get(obj, (None, None));
		if cmd and type(cmd) is dict:
			cmd = cmd.get(obj.GetValue())
		if cmd:
			print(cmd)
			os.system(cmd)

	def OnLaunchPmap(self, event):
		self.OnSelector_obj(self.button_launch_pmap)

	def OnKillPmap(self, event):
		self.OnSelector_obj(self.button_kill_pmap)

	def OnPointMapUpdate(self, event):
		sdic = self.selector.get('pmap', {})
		if sdic.get('launched'):
			self.OnKillPmap(None)
			self.OnLaunchPmap(None)

	#
	# Data Tab
	#

	#
	# Common Utils
	#
	def OnSelecotr(self, event):
		self.OnSelector_obj(self, event.GetEventObject())

	def OnSelector_obj(self, obj):
		pfs = ('button_launch_', 'button_kill_', 'button_', 'checkbox_')
		vs = (True, False, None, None)
		pfinf = dict(zip(pfs, vs))

		(pf, key) = self.obj_name_split(obj, pfs)
		if key is None:
			return
		v = pfinf.get(pf)
		if v is None and getattr(obj, 'GetValue', None):
			v = obj.GetValue()
		if v is None:
			return
		if self.OnSelector_name(key, v) is None:
			if getattr(obj, 'SetValue', None):
				set_val(obj, not v)

	def OnSelector_name(self, key, v):
		sdic = self.selector.get(key)
		if sdic is None:
			return None
		if v:
			sels = eval(sdic.get('sel', 'None'))
			if sels is None or sels == []:
				return None
			for sel in sels:
				name = sdic.get(sel)
				if name is None:
					continue
				obj = self.obj_get('button_launch_' + name)
				self.OnLaunch_obj(obj)
			sdic['launched'] = sels
		else:
			sels = sdic.get('launched', [])
			for sel in sels:
				name = sdic.get(sel)
				if name is None:
					continue
				kill_obj = self.obj_get('button_kill_' + name)
				self.OnKill_kill_obj(kill_obj)
			sdic['launched'] = None
		return True

	def set_param_panel(self, obj, parent):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		panel = ParamPanel(parent, frame=self, pdic=pdic, gdic=gdic, prm=prm)
		szr = wx.BoxSizer(wx.VERTICAL)
		szr.Add(panel, 0, wx.EXPAND)
		parent.SetSizer(szr)
		k = 'ext_toggle_enables'
		gdic[ k ] = gdic.get(k, []) + [ panel ]

	def OnConfig(self, event):
		self.OnHyperlinked_obj(event.GetEventObject())

	def add_params(self, params):
		for prm in params:
			if 'topic' in prm and 'msg' in prm:
				klass_msg = globals()[ prm['msg'] ]
				prm['pub'] = rospy.Publisher(prm['topic'], klass_msg, queue_size=10)
		self.params += params

	def gdic_get_1st(self, dic):
		gdic = dic.get('gui', {})
		gdic['update_func'] = self.update_func
		return gdic

	def get_cfg_obj(self, obj):
		return get_top( [ k for (k,v) in self.config_dic.items() if v['obj'] is obj ] )

	def add_cfg_info(self, cfg_obj, obj, name, pdic, gdic, run_disable, prm):
		self.config_dic[ cfg_obj ] = { 'obj':obj , 'name':name , 'pdic':pdic , 'gdic':gdic, 
					       'run_disable':run_disable , 'param':prm }

	def get_param(self, prm_name):
		return get_top( [ prm for prm in self.params if prm['name'] == prm_name ] )

	def setup_config_param_pdic(self):
		for info in self.config_dic.values():
			if 'param' in info and info['pdic'] is None:
				self.setup_create_pdic(info)

	def setup_create_pdic(self, targ_info):
		prm = targ_info.get('param')
		info = get_top( [ info for info in self.config_dic.values() if info.get('param', None) == prm and info['pdic'] ] )
		if info:
			targ_info['pdic'] = info['pdic']
			return
		pdic = {}
		if prm:
			for var in prm['vars']:
				pdic[ var['name'] ] = var['v']
		targ_info['pdic'] = pdic
		self.load_dic[ targ_info['name'] ] = pdic

	def obj_to_cmd_dic(self, obj):
		return get_top( [ cmd_dic for cmd_dic in self.all_cmd_dics if obj in cmd_dic ] )

	def obj_to_cmd_dic_cmd_proc(self, obj):
		cmd_dic = self.obj_to_cmd_dic(obj)
		if cmd_dic is None:
			return (None, None, None)
		(cmd, proc) = cmd_dic.get(obj, (None, None))
		return (cmd_dic, cmd, proc)

	def OnLaunch(self, event):
		self.OnLaunch_obj(event.GetEventObject())

	def OnLaunch_obj(self, obj):
		obj = self.alias_grp_top_obj(obj)
		self.alias_sync(obj, v=True)

		add_args = self.obj_to_add_args(obj)
		#print 'add_args', add_args
		if add_args is False:
			return
		if self.is_boot(obj):
			wx.MessageBox('Already, booted')
			return

		key = self.obj_key_get(obj, ['button_launch_'])
		if not key:
			return
		tc = self.obj_get('text_ctrl_' + key) 
		path = tc.GetValue() if tc else None

		if tc and not path:
			return

		(cmd_dic, cmd, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if cmd_dic is None or cmd is None:
			return

		if path:
			add_args = ( add_args if add_args else [] ) + path.split(',')

		proc = self.launch_kill(True, cmd, proc, add_args, obj=obj)
		cmd_dic[obj] = (cmd, proc)

		self.toggle_enable_obj(obj)

	def OnKill(self, event):
		self.OnKill_kill_obj(event.GetEventObject())

	def OnKill_kill_obj(self, kill_obj):
		kill_obj = self.alias_grp_top_obj(kill_obj)
		self.alias_sync(kill_obj, v=False)

		key = self.obj_key_get(kill_obj, ['button_kill_'])
		if not key:
			return
		obj = self.obj_get('button_launch_' + key)
		(cmd_dic, cmd, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if cmd_dic is None or cmd is None:
			return

		# ROSBAG Record modify
		sigint = (key == 'rosbag_record')

		proc = self.launch_kill(False, cmd, proc, sigint=sigint, obj=obj)
		cmd_dic[obj] = (cmd, proc)

		self.toggle_enable_obj(obj)
		self.stat_label_off(obj)

	def OnLaunchKill(self, event):
		self.OnLaunchKill_obj(event.GetEventObject())

	def OnLaunchKill_obj(self, obj):
		v = obj.GetValue()
		add_args = self.obj_to_add_args(obj)
		if add_args is False:
			set_val(obj, not v)
			return
		(cmd_dic, _, proc_bak) = self.obj_to_cmd_dic_cmd_proc(obj)
		self.launch_kill_proc(obj, cmd_dic, add_args=add_args)
		(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if proc != proc_bak:
			self.toggle_enable_obj(obj)

	def OnPauseRosbagPlay(self, event):
		pause_obj = event.GetEventObject()
		pause_obj = self.alias_grp_top_obj(pause_obj)

		key = self.obj_key_get(pause_obj, ['button_pause_'])
		if not key:
			return
		obj = self.obj_get('button_launch_' + key)
		(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if proc:
			proc.stdin.write(' ')

	def OnRef(self, event):
		btn_ref_inf = {
			'point_cloud'	: { 'path_type' : 'multi' },
			'pmap'		: { 'path_type' : 'multi' },
			'vector_map'	: { 'path_type' : 'multi' },
			'calibration'	: { 'path_type' : 'dir'	  },
			'rosbag_record' : { 'path_type' : 'save'  } }
		obj = event.GetEventObject()
		key = self.obj_key_get(obj, [ 'button_ref_' ])
		if key is None:
			return
		tc = self.obj_get('text_ctrl_' + key)
		if tc is None:
			return
		if file_dialog(self, tc, btn_ref_inf.get(key, {})) == wx.ID_OK:
			self.alias_sync(tc)

	def OnAliasSync(self, event):
		obj = event.GetEventObject()
		self.alias_sync(obj)

	def alias_sync(self, obj, v=None):
		en = None
		if getattr(obj, 'IsEnabled', None):
			en = obj.IsEnabled()
		grp = self.alias_grp_get(obj)
		if getattr(obj, 'GetValue', None):
			v = obj.GetValue()
		for o in grp:
			if o is obj:
				continue
			
			if en is not None and o.IsEnabled() != en and not self.is_toggle_button(o):
				o.Enable(en)
			if v is not None and getattr(o, 'SetValue', None):
				o.SetValue(v)
				if getattr(o, 'SetInsertionPointEnd', None):
					o.SetInsertionPointEnd()

	def alias_grp_top_obj(self, obj):
		return get_top(self.alias_grp_get(obj), obj)

	def alias_grp_get(self, obj):
		return get_top([ grp for grp in self.alias_grps if obj in grp ], [])

	def create_tree(self, parent, items, tree, item, cmd_dic):
		name = items.get('name', '')
		if tree is None:
			style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
			tree = CT.CustomTreeCtrl(parent, wx.ID_ANY, style=style)
			item = tree.AddRoot(name, data=tree)
		else:
			ct_type = 1 if 'cmd' in items else 0 # 1:checkbox type
			item = tree.AppendItem(item, name, ct_type=ct_type)
			if 'cmd' in items:
				cmd_dic[item] = (items['cmd'], None)

			if 'param' in items:
				prm = self.get_param(items.get('param'))
				gdic = self.gdic_get_1st(items)
				self.add_config_link_tree_item(item, name, gdic, prm)

		for sub in items.get('subs', []):
			self.create_tree(parent, sub, tree, item, cmd_dic)
		return tree

	def add_config_link_tree_item(self, item, name, gdic, prm):
		pdic = self.load_dic.get(name, {})
		self.load_dic[name] = pdic
		self.add_cfg_info(item, item, name, pdic, gdic, False, prm)
		item.SetHyperText()

	def launch_kill_proc(self, obj, cmd_dic, add_args=None):
		if obj not in cmd_dic:
			set_val(obj, False)
			print('not implemented.')
			return
		v = obj.GetValue()
		if v and self.is_boot(obj):
			wx.MessageBox('Already, booted')
			set_val(obj, not v)
			return

		(cmd, proc) = cmd_dic[obj]
		if not cmd:
			set_val(obj, False)

		proc = self.launch_kill(v, cmd, proc, add_args, obj=obj)

		cfg_obj = self.get_cfg_obj(obj)
		if cfg_obj and self.config_dic[ cfg_obj ]['run_disable']:
			cfg_obj.Enable(not v)

		cmd_dic[obj] = (cmd, proc)
		if not v:
			self.stat_label_off(obj)

	def kill_all(self):
		all = self.all_procs[:] # copy
		for proc in all:
			self.kill_proc(proc)

	def kill_proc(self, proc):
		(cmd_dic, obj) = self.proc_to_cmd_dic_obj(proc)
		self.kill_obj(obj, cmd_dic, proc)

	def kill_obj(self, obj, cmd_dic=None, proc=None):
		key = self.obj_key_get(obj, [ 'button_launch_' ])
		if key:
			self.OnKill_kill_obj(self.obj_get('button_kill_' + key))
			return
		set_val(obj, False)
		if cmd_dic is None:
			cmd_dic = self.obj_to_cmd_dic(obj)
		v = cmd_dic.get(obj)
		if v is None:
			return
		(cmd, proc) = (v[0], proc) if proc else v
		cmd_dic[ obj ] = (cmd, None)
		self.launch_kill(False, 'dmy', proc, obj=obj)
		self.stat_label_off(obj)

	def proc_to_cmd_dic_obj(self, proc):
		for cmd_dic in self.all_cmd_dics:
			obj = get_top( [ obj for (obj, v) in cmd_dic.items() if proc in v ] )
			if obj:
				return (cmd_dic, obj)
		return (None, None)

	def launch_kill(self, v, cmd, proc, add_args=None, sigint=False, obj=None):
		msg = None
		msg = 'already launched.' if v and proc else msg
		msg = 'already terminated.' if not v and proc is None else msg
		msg = 'cmd not implemented.' if not cmd else msg
		if msg is not None:
			print(msg)
			return proc

		if v:
			args = shlex.split(cmd)
			if add_args:
				args += add_args
			print(args) # for debug
			proc = subprocess.Popen(args, stdin=subprocess.PIPE)
			self.all_procs.append(proc)
			self.all_procs_nodes[ proc ] = self.nodes_dic.get(obj, [])
		else:
			terminate_children(proc, sigint)
			terminate(proc, sigint)
			proc.wait()
			if proc in self.all_procs:
				self.all_procs.remove(proc)
				self.all_procs_nodes.pop(proc, None)
			proc = None
		return proc

	def is_boot(self, obj):
		nodes = self.nodes_dic.get(obj)
		if nodes is None:
			return False
		boot_nodes = reduce(lambda a,b:a+b, [[]] + self.all_procs_nodes.values())
		boot_nodes_set = set(boot_nodes)
		return nodes and boot_nodes_set.issuperset(nodes)

	def nodes_dic_get(self):
		print 'creating item node list ',
		nodes_dic = {}
		#for cmd_dic in self.all_cmd_dics:
		for cmd_dic in [ self.main_cmd, self.computing_cmd, self.data_cmd, self.sensing_cmd ]:
			sys.stdout.write('.')
			sys.stdout.flush()
			for (obj, (cmd, _)) in cmd_dic.items():
				if not cmd:
					continue
				nodes = []
				cmd = shlex.split(cmd)
				cmd2 = cmd
				if cmd[0] == 'sh' and cmd[1] == '-c':
					cmd2 = cmd[2].split(' ') + cmd[3:] # split ' '
				if cmd2[0] == 'roslaunch':
					add_args = self.obj_to_add_args(obj)
					if add_args:
						cmd2 += add_args
					cmd2.insert(1, '--node')
					if cmd[0] == 'sh' and cmd[1] == '-c':
						cmd[2] = ' '.join(cmd2)
					nodes = self.roslaunch_to_nodes(cmd)
				elif cmd2[0] == 'rosrun':
					nodes = [ '/' + cmd2[2] ]
				nodes_dic[ obj ] = nodes
		print ''
		return nodes_dic

	def roslaunch_to_nodes(self, cmd):
		try:
			return subprocess.check_output(cmd).strip().split('\n')
		except subprocess.CalledProcessError:
			return []

	def modal_dialog(self, lst, title=''):
		(lbs, cmds) = zip(*lst)
		dlg = MyDialog(self, lbs=lbs)
		dlg.SetTitle(title)
		r = dlg.ShowModal()
		ok = (0 <= r and r < len(cmds))
		return cmds[r] if ok else None

	def get_autoware_dir(self):
		dir = os.path.abspath(os.path.dirname(__file__)) + '/../../../../../../'
		return os.path.abspath(dir)

	def get_static_bitmap(self, parent, filename, scale):
		bm = self.get_bitmap(filename, scale)
		return wx.StaticBitmap(parent, wx.ID_ANY, bm)

	def get_bitmap(self, filename, scale):
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		bm = wx.Bitmap(dir + filename, wx.BITMAP_TYPE_ANY)
		(w, h) = bm.GetSize()
		img = wx.ImageFromBitmap(bm)
		img = img.Scale(w * scale, h * scale, wx.IMAGE_QUALITY_HIGH)
		bm = wx.BitmapFromImage(img)
		return bm

	def load_yaml(self, filename, def_ret=None):
		return load_yaml(filename, def_ret)

	def toggle_enable_obj(self, obj):
		objs = []
		pfs = [ 'button_launch_', 'button_kill_', 'button_pause_', 'button_ref_', 'text_ctrl_' ]
		key = self.obj_key_get(obj, pfs)
		if key:
			objs += self.key_objs_get(pfs, key)
			
		(_, gdic, _) = self.obj_to_pdic_gdic_prm(obj)
		if gdic:
			objs += [ (eval(e) if type(e) is str else e) for e in gdic.get('ext_toggle_enables', []) ]

		self.toggle_enables(objs)

	def toggle_enables(self, objs):
		for obj in objs:
			if getattr(obj, 'IsEnabled', None):
				obj.Enable(not obj.IsEnabled())
				self.alias_sync(obj)

	def is_toggle_button(self, obj):
		return self.name_get(obj).split('_')[0] == 'button' and getattr(obj, 'GetValue', None)

	def obj_name_split(self, obj, pfs):
		name = self.name_get(obj)
		if name is None:
			return (None, None)
		return get_top( [ ( name[:len(pf)], name[len(pf):] ) for pf in pfs if name[:len(pf)] == pf ] )

	def obj_key_get(self, obj, pfs):
		name = self.name_get(obj)
		if name is None:
			return None
		return get_top( [ name[len(pf):] for pf in pfs if name[:len(pf)] == pf ] )

	def key_objs_get(self, pfs, key):
		return [ self.obj_get(pf + key) for pf in pfs if self.obj_get(pf + key) ]

	def name_get(self, obj):
		return get_top( [ nm for nm in dir(self) if getattr(self, nm) is obj ] )

	def val_get(self, name):
		obj = self.obj_get(name)
		if obj is None:
			return None
		return obj.GetValue() if getattr(obj, 'GetValue', None) else None

	def obj_get(self, name):
		return getattr(self, name, None)

	def key_get(self, dic, val):
		return get_top( [ k for (k,v) in dic.items() if v == val ] )

class MyDialog(rtmgr.MyDialog):
	def __init__(self, *args, **kwds):
		lbs = kwds.pop('lbs')
		rtmgr.MyDialog.__init__(self, *args, **kwds)

		self.radio_box.Destroy()
		self.radio_box = wx.RadioBox(self.panel_2, wx.ID_ANY, "", choices=lbs, majorDimension=0, style=wx.RA_SPECIFY_ROWS)

		rtmgr.MyDialog.__set_properties(self)
		rtmgr.MyDialog.__do_layout(self)

	def __set_properties(self):
		pass

	def __do_layout(self):
		pass

	def OnOk(self, event):
		ret = self.radio_box.GetSelection()
		self.EndModal(ret)

	def OnCancel(self, event):
		self.EndModal(-1)

class ParamPanel(wx.Panel):
	def __init__(self, *args, **kwds):
		self.frame = kwds.pop('frame')
		self.pdic = kwds.pop('pdic')
		self.gdic = kwds.pop('gdic')
		self.prm = kwds.pop('prm')
		wx.Panel.__init__(self, *args, **kwds)

		obj = get_top( [ v.get('obj') for (cfg_obj, v) in self.frame.config_dic.items() if v.get('param') is self.prm ] )
		(_, _, proc) = self.frame.obj_to_cmd_dic_cmd_proc(obj)

		hszr = None
		self.vps = []
		self.tmp_msg = None
		szr = wx.BoxSizer(wx.VERTICAL)

		topic_szrs = (None, None)
		for var in self.prm.get('vars'):
			name = var.get('name')
			if name not in self.gdic:
				self.gdic[ name ] = {}
			gdic_v = self.gdic.get(name)
			if gdic_v.get('func'):
				continue

			v = self.pdic.get(name, var.get('v'))

			vp = VarPanel(self, var=var, v=v, update=self.update)
			self.vps.append(vp)

			gdic_v['var'] = vp
			gdic_v['func'] = vp.get_v
			prop = gdic_v.get('prop', 0)
			border = gdic_v.get('border', 0)
			flag = wx_flag_get(gdic_v.get('flags', []))

			do_category = 'no_category' not in gdic_v.get('flags', [])
			if do_category and self.in_msg(var):
				bak = (szr, hszr)
				(szr, hszr) = topic_szrs
				if szr is None:
					szr = static_box_sizer(self, 'topic : ' + self.prm.get('topic'))
					bak[0].Add(szr, 0, wx.EXPAND | wx.ALL, 4)
			targ_szr = szr
			if vp.has_slider or vp.kind =='path':
				hszr = None if hszr else hszr
				flag |= wx.EXPAND
			else:
				if hszr is None:
					hszr = wx.BoxSizer(wx.HORIZONTAL)
					szr.Add(hszr, 0, wx.EXPAND)
				flag |= wx.ALIGN_CENTER_VERTICAL
				targ_szr = hszr

			if do_category and 'rosparam' in var:
				rp_szr = static_box_sizer(self, 'rosparam : ' + var.get('rosparam'))
				targ_szr.Add(rp_szr, 0, wx.EXPAND | wx.ALL, 4)
				targ_szr = rp_szr

			targ_szr.Add(vp, prop, flag, border)

			if 'nl' in gdic_v.get('flags', []):
				hszr = None

			if do_category and self.in_msg(var):
				topic_szrs = (szr, hszr)
				(szr, hszr) = bak

			if not self.in_msg(var) and var.get('rosparam'):
				k = 'ext_toggle_enables'
				self.gdic[ k ] = self.gdic.get(k, []) + [ vp ]
				vp.Enable(proc is None)

		self.SetSizer(szr)
		self.update()

	def update(self):
		update_func = self.gdic.get('update_func')
		if update_func:
			update_func(self.pdic, self.gdic, self.prm)

	def detach_func(self):
		for var in self.prm.get('vars'):
			name = var.get('name')
			gdic_v = self.gdic.get(name, {})
			if 'func' in gdic_v:
				del gdic_v['func']

	def in_msg(self, var):
		if 'topic' not in self.prm or 'msg' not in self.prm:
			return False
		if self.tmp_msg is None:
			klass_msg = globals().get( self.prm.get('msg') )
			if klass_msg is None:
				return False
			self.tmp_msg = klass_msg()
		(obj, attr) = msg_path_to_obj_attr(self.tmp_msg, var.get('name'))
		return obj and attr in obj.__slots__

class VarPanel(wx.Panel):
	def __init__(self, *args, **kwds):
		self.var = kwds.pop('var')
		v = kwds.pop('v')
		self.update = kwds.pop('update')
		wx.Panel.__init__(self, *args, **kwds)

		self.min = self.var.get('min')
		self.max = self.var.get('max')
		self.has_slider = self.min is not None and self.max is not None

		label = self.var.get('label', '')
		self.kind = self.var.get('kind')
		if self.kind == 'radio_box':
			choices = self.var.get('choices', [])
			self.obj = wx.RadioBox(self, wx.ID_ANY, label, choices=choices, majorDimension=0, style=wx.RA_SPECIFY_ROWS)
			self.choices_sel_set(v)
			self.Bind(wx.EVT_RADIOBOX, self.OnUpdate, self.obj)
			return
		if self.kind == 'menu':
			choices = self.var.get('choices', [])
			slef.obj = wx.Choice(self, wx.ID_ANY, choices=choices)
			self.choices_sel_set(v)
			self.Bind(wx.EVT_CHOICE, self.OnUpdate, self.obj)
			return
		if self.kind == 'checkbox':
			self.obj = wx.CheckBox(self, wx.ID_ANY, label)
			self.obj.SetValue(v)
			self.Bind(wx.EVT_CHECKBOX, self.OnUpdate, self.obj)
			return
		if self.kind == 'toggle_button':
			self.obj = wx.ToggleButton(self, wx.ID_ANY, label)
			self.obj.SetValue(v)
			self.Bind(wx.EVT_TOGGLEBUTTON, self.OnUpdate, self.obj)
			return
		if self.kind == 'hide':
			self.Hide()
			return

		szr = wx.BoxSizer(wx.HORIZONTAL)

		lb = wx.StaticText(self, wx.ID_ANY, label)
		flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
		szr.Add(lb, 0, flag, 4)

		self.tc = wx.TextCtrl(self, wx.ID_ANY, str(v), style=wx.TE_PROCESS_ENTER)
		self.Bind(wx.EVT_TEXT_ENTER, self.OnUpdate, self.tc)

		if self.kind is None:
			if self.has_slider:
				self.w = self.max - self.min
				vlst = [ v, self.min, self.max, self.var['v'] ]
				self.is_float = len( [ v_ for v_ in vlst if type(v_) is not int ] ) > 0
				self.int_max = 1000 if self.is_float else self.max
				self.int_min = 0 if self.is_float else self.min

				self.slider = wx.Slider(self, wx.ID_ANY, self.get_int_v(), self.int_min, self.int_max)
				self.Bind(wx.EVT_COMMAND_SCROLL, self.OnScroll, self.slider)
				self.slider.SetMinSize((82, 27))
				szr.Add(self.slider, 1, wx.LEFT | wx.RIGHT | wx.ALIGN_CENTER_VERTICAL, 4)
			else:
				self.is_float = type(self.var['v']) is not int
				self.tc.SetMinSize((40,27))

		flag = wx.ALIGN_CENTER_VERTICAL
		prop = 1 if self.kind == 'path' else 0
		szr.Add(self.tc, prop, flag, 4)

		if self.kind == 'path':
			self.ref = wx.Button(self, wx.ID_ANY, 'Ref')
			self.Bind(wx.EVT_BUTTON, self.OnRef, self.ref)
			self.ref.SetMinSize((40,29))
			szr.Add(self.ref, 0, flag, 4)

		if self.has_slider:
			vszr = wx.BoxSizer(wx.VERTICAL)
			vszr.Add( self.create_bmbtn("inc.png", self.OnIncBtn) )
			vszr.Add( self.create_bmbtn("dec.png", self.OnDecBtn) )
			szr.Add(vszr, 0, wx.ALIGN_CENTER_VERTICAL)

		self.SetSizer(szr)

	def create_bmbtn(self, filename, hdr):
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		bm = wx.Bitmap(dir + filename, wx.BITMAP_TYPE_ANY)
		style = wx.BORDER_NONE | wx.BU_EXACTFIT
		obj = wx.lib.buttons.GenBitmapButton(self, wx.ID_ANY, bm, style=style)
		self.Bind(wx.EVT_BUTTON, hdr, obj)
		return obj

	def get_v(self):
		if self.kind in [ 'radio_box', 'menu' ]:
			return self.choices_sel_get()
		if self.kind in [ 'checkbox', 'toggle_button' ]:
			return self.obj.GetValue()
		if self.kind == 'hide':
			return self.var.get('v')
		if self.kind == 'path':
			return str(self.tc.GetValue())

		if not self.has_slider and self.tc.GetValue() == '':
			return ''
		return self.get_tc_v()

	def get_tc_v(self):
		s = self.tc.GetValue()
		v = float(s) if self.is_float else int(s)
		if self.has_slider:
			v = self.min if v < self.min else v
			v = self.max if v > self.max else v
		self.tc.SetValue(adjust_num_str(str(v)))
		return v

	def get_int_v(self):
		v = self.get_tc_v()
		if self.is_float:
			v = int( self.int_max * (v - self.min) / self.w if self.w != 0 else 0 )
		return v

	def OnScroll(self, event):
		iv = self.slider.GetValue()
		s = str(iv)
		if self.is_float:
			v = self.min + float(self.w) * iv / self.int_max
			s = str(Decimal(v).quantize(Decimal(str(self.get_step()))))
		self.tc.SetValue(s)
		self.update()

	def OnIncBtn(self, event):
		step = self.get_step()
		self.add_v(step)

	def OnDecBtn(self, event):
		step = self.get_step()
		self.add_v(-step)

	def get_step(self):
		step = self.var.get('step')
		return step if step else 0.01 if self.is_float else 1

	def add_v(self, step):
		ov = self.get_v()
		self.tc.SetValue(str(ov + step))
		v = self.get_v()
		if v != ov:
			self.slider.SetValue(self.get_int_v())
			self.update()

	def OnUpdate(self, event):
		if self.has_slider:
			self.slider.SetValue(self.get_int_v())
		self.update()

	def OnRef(self, event):
		if file_dialog(self, self.tc, self.var) == wx.ID_OK:
			self.update()

	def choices_sel_get(self):
		return self.obj.GetStringSelection() if self.var.get('choices_type') == 'str' else self.obj.GetSelection()

	def choices_sel_set(self, v):
		if self.var.get('choices_type') == 'str':
			self.obj.SetStringSelection(v)
		else:
			self.obj.SetSelection(v)

class MyDialogParam(rtmgr.MyDialogParam):
	def __init__(self, *args, **kwds):
		pdic = kwds.pop('pdic')
		self.pdic_bak = pdic.copy()
		gdic = kwds.pop('gdic')
		prm = kwds.pop('prm')
		rtmgr.MyDialogParam.__init__(self, *args, **kwds)

		parent = self.panel_v
		frame = self.GetParent()
		self.panel = ParamPanel(parent, frame=frame, pdic=pdic, gdic=gdic, prm=prm)
		szr = wx.BoxSizer(wx.VERTICAL)
		szr.Add(self.panel, 1, wx.EXPAND)
		parent.SetSizer(szr)

		self.SetTitle(prm.get('name', ''))
		(w,h) = self.GetSize()
		(w2,_) = szr.GetMinSize()
		w2 += 20
		if w2 > w:
			self.SetSize((w2,h))

	def OnOk(self, event):
		self.panel.update()
		self.panel.detach_func()
		self.EndModal(0)

	def OnCancel(self, event):
		self.panel.pdic.update(self.pdic_bak) # restore
		self.panel.detach_func()
		self.panel.update()
		self.EndModal(-1)

class MyDialogLaneStop(rtmgr.MyDialogLaneStop):
	def __init__(self, *args, **kwds):
		self.pdic = kwds.pop('pdic')
		self.gdic = kwds.pop('gdic')
		self.prm = kwds.pop('prm')
		rtmgr.MyDialogLaneStop.__init__(self, *args, **kwds)
		self.frame = self.GetParent()

	def update(self):
		update_func = self.gdic.get('update_func')
		if update_func:
			update_func(self.pdic, self.gdic, self.prm)

	def OnTrafficRedLight(self, event):
		self.pdic['traffic_light'] = 0
		self.update()
		
	def OnTrafficGreenLight(self, event):
		self.pdic['traffic_light'] = 1
		self.update()

	def OnOk(self, event):
		self.EndModal(0)

	def OnCancel(self, event):
		self.EndModal(-1)

class MyDialogNdtSlam(rtmgr.MyDialogNdtSlam):
	def __init__(self, *args, **kwds):
		self.pdic = kwds.pop('pdic')
		self.pdic_bak = self.pdic.copy()
		self.gdic = kwds.pop('gdic')
		self.prm = kwds.pop('prm')
		rtmgr.MyDialogNdtSlam.__init__(self, *args, **kwds)

		parent = self.panel_v
		frame = self.GetParent()
		self.panel = ParamPanel(parent, frame=frame, pdic=self.pdic, gdic=self.gdic, prm=self.prm)
		szr = wx.BoxSizer(wx.VERTICAL)
		szr.Add(self.panel, 1, wx.EXPAND)
		parent.SetSizer(szr)

		self.update_filename()
		self.klass_msg = ConfigNdtSlamOutput
		self.pub = rospy.Publisher('/config/ndt_slam_output', self.klass_msg, queue_size=10)

	def update_filename(self):
		tc = self.text_ctrl_path
		path = tc.GetValue()
		(dn, fn) = os.path.split(path)
		now = datetime.datetime.now()
		fn = 'autoware-%02d%02d%02d.pcd' % (
			now.year % 100, now.month, now.day)
		path = os.path.join(dn, fn)
		set_path(tc, path)

	def OnRef(self, event):
		tc = self.text_ctrl_path
		file_dialog(self, tc, { 'path_type' : 'save' } )

	def OnRadio(self, event):
		v = self.radio_btn_filter_resolution.GetValue()
		tc = self.text_ctrl_filter_resolution
		tc.Enable(v)

	def OnPcdOutput(self, event):
		tc = self.text_ctrl_filter_resolution
		v = tc.GetValue() if self.radio_btn_filter_resolution.GetValue() else '0.0'
		msg = self.klass_msg()
		msg.filename = self.text_ctrl_path.GetValue()
		msg.filter_res = float(v)
		self.pub.publish(msg)
		
	def OnOk(self, event):
		self.panel.update()
		self.panel.detach_func()
		self.EndModal(0)

	def OnCancel(self, event):
		self.panel.pdic.update(self.pdic_bak) # restore
		self.panel.detach_func()
		self.panel.update()
		self.EndModal(-1)

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = MyFrame(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1

class MyDialogRosbagRecord(rtmgr.MyDialogRosbagRecord):
	def __init__(self, *args, **kwds):
		self.cmd_dic = kwds.pop('cmd_dic')
		rtmgr.MyDialogRosbagRecord.__init__(self, *args, **kwds)
		self.cbs = []
		self.refresh()
		self.parent = self.GetParent()
		self.cmd_dic[ self.button_start ] = ('rosbag record', None)

	def OnRef(self, event):
		tc = self.text_ctrl
		file_dialog(self, tc, { 'path_type' : 'save' } )

	def OnStart(self, event):
		key_obj = self.button_start
		path = self.text_ctrl.GetValue()
		if path == '':
			print('path=""')
			return
		topic_opt = []
		if self.cbs[0].GetValue(): # 'All'
			topic_opt = [ '-a' ]
		else:
			for obj in self.cbs:
				if obj.GetValue():
					topic_opt += [ obj.GetLabel() ]
		if topic_opt == []:
			print('topic=[]')
			return
		args = topic_opt + [ '-O', path ]

		(cmd, proc) = self.cmd_dic[ key_obj ]
		proc = self.parent.launch_kill(True, cmd, proc, add_args=args, obj=key_obj)
		self.cmd_dic[ key_obj ] = (cmd, proc)

	def OnStop(self, event):
		key_obj = self.button_start
		(cmd, proc) = self.cmd_dic[ key_obj ]
		proc = self.parent.launch_kill(False, cmd, proc, sigint=True, obj=key_obj)
		self.cmd_dic[ key_obj ] = (cmd, proc)
		self.Hide()

	def OnRefresh(self, event):
		self.refresh()

	def refresh(self):
		lst = [ 'all' ] + subprocess.check_output([ 'rostopic', 'list' ]).strip().split('\n')
		panel = self.panel_1
		szr = self.sizer_topic
		for obj in self.cbs:
			szr.Remove(obj)
			obj.Destroy()
		self.cbs = []
		for topic in lst:
			obj = wx.CheckBox(panel, wx.ID_ANY, topic)
			bdr = 4 if topic == 'All' else 4 * 4
			szr.Add(obj, 0, wx.LEFT, bdr)
			self.cbs.append(obj)
		szr.Layout()
		panel.SetVirtualSize(szr.GetMinSize())
		self.update_filename();

	def update_filename(self):
		tc = self.text_ctrl
		path = tc.GetValue()
		(dn, fn) = os.path.split(path)
		now = datetime.datetime.now()
		fn = 'autoware-%04d%02d%02d%02d%02d%02d.rosbag' % (
			now.year, now.month, now.day, now.hour, now.minute, now.second)
		path = os.path.join(dn, fn)
		set_path(tc, path)

def file_dialog(parent, tc, path_inf_dic={}):
	path = tc.GetValue()
	path = get_top(path.split(','), path)
	(dn, fn) = os.path.split(path)
	path_type = path_inf_dic.get('path_type')
	if path_type == 'dir':
		fns = path_inf_dic.get('filenames')
		if type(fns) is str and fns[-5:] == '.yaml':
			fns = load_yaml(fns)
			if type(fns) is not list:
				fns = None
			path_inf_dic['filenames'] = fns
		dlg = wx.DirDialog(parent, defaultPath=path)
	else:
		st_dic = { 'save' : wx.FD_SAVE, 'multi' : wx.FD_MULTIPLE }
		dlg = wx.FileDialog(parent, defaultDir=dn, defaultFile=fn, 
				    style=st_dic.get(path_type, wx.FD_DEFAULT_STYLE))
	ret = dlg.ShowModal()
	if ret == wx.ID_OK:
		path = ','.join(dlg.GetPaths()) if path_type == 'multi' else dlg.GetPath()
		if path_type == 'dir' and fns:
			path = ','.join([ path + '/' + fn for fn in fns ])
		set_path(tc, path)
	dlg.Destroy()
	return ret

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

def terminate_children(proc, sigint=False):
	for child in psutil.Process(proc.pid).get_children():
		terminate(child, sigint)

def terminate(proc, sigint=False):
	if sigint:
		proc.send_signal(signal.SIGINT)
	else:
		proc.terminate()

def static_box_sizer(parent, s, orient=wx.VERTICAL):
	sb = wx.StaticBox(parent, wx.ID_ANY, s)
	sb.Lower()
	return wx.StaticBoxSizer(sb, orient)

def wx_flag_get(flags):
	dic = { 'top' : wx.TOP, 'bottom' : wx.BOTTOM, 'left' : wx.LEFT, 'right' : wx.RIGHT, 
		'all' : wx.ALL, 'expand' : wx.EXPAND, 'fixed_minsize' : wx.FIXED_MINSIZE,
		'center_v' : wx.ALIGN_CENTER_VERTICAL, 'center_h' : wx.ALIGN_CENTER_HORIZONTAL }
	lst = [ dic.get(f) for f in flags if f in dic ]
	return reduce(lambda a,b : a+b, [0] + lst)

def msg_path_to_obj_attr(msg, path):
	lst = path.split('.')
	obj = msg
	for attr in lst[:-1]:
		obj = getattr(obj, attr, None)
	return (obj, lst[-1])

def str_to_rosval(str, type_str, def_ret=None):
	cvt_dic = {
		'int8':int , 'int16':int , 'int32':int ,
		'uint8':int , 'uint16':int , 'uint32':int ,
		'int64':long , 'uint64':long,
		'float32':float, 'float64':float,
	}
	t = cvt_dic.get(type_str)
	return t(str) if t else def_ret

def set_path(tc, v):
	tc.SetValue(v)
	tc.SetInsertionPointEnd()

def set_val(obj, v):
	func = getattr(obj, 'SetValue', getattr(obj, 'Check', None))
	if func:
		func(v)
		obj_refresh(obj)

def obj_refresh(obj):
	if type(obj) is wx.lib.agw.customtreectrl.GenericTreeItem:
		while obj.GetParent():
			obj = obj.GetParent()
		tree = obj.GetData()
		tree.Refresh()

def get_top(lst, def_ret=None):
	return lst[0] if len(lst) > 0 else def_ret

def adjust_num_str(s):
	if '.' in s:
		while s[-1] == '0':
			s = s[:-1]
		if s[-1] == '.':
			s = s[:-1]
	return s

def prn_dict(dic):
	for (k,v) in dic.items():
		print (k, ':', v)

if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()

# EOF
