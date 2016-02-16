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
import gtk
import gettext
import os
import re
import sys
import fcntl
import threading
import Queue
import time
import socket
import struct
import shlex
import signal
import subprocess
import psutil
import pty
import yaml
import datetime
import syslog
import rtmgr
import rospy
import std_msgs.msg
from std_msgs.msg import Bool
from decimal import Decimal
from runtime_manager.msg import ConfigRcnn
from runtime_manager.msg import ConfigCarDpm
from runtime_manager.msg import ConfigPedestrianDpm
from runtime_manager.msg import ConfigNdt
from runtime_manager.msg import ConfigNdtMapping
from runtime_manager.msg import ConfigNdtMappingOutput
from runtime_manager.msg import ConfigWaypointFollower
from runtime_manager.msg import ConfigTwistFilter
from runtime_manager.msg import ConfigVelocitySet
from runtime_manager.msg import ConfigCarKf
from runtime_manager.msg import ConfigPedestrianKf
from runtime_manager.msg import ConfigLaneRule
from runtime_manager.msg import ConfigLaneSelect
from runtime_manager.msg import ConfigLaneStop
from runtime_manager.msg import ConfigCarFusion
from runtime_manager.msg import ConfigPedestrianFusion
from tablet_socket.msg import mode_cmd
from tablet_socket.msg import gear_cmd
from tablet_socket.msg import Waypoint
from tablet_socket.msg import route_cmd
from ndt_localizer.msg import ndt_stat
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from runtime_manager.msg import accel_cmd
from runtime_manager.msg import steer_cmd
from runtime_manager.msg import brake_cmd
from runtime_manager.msg import indicator_cmd
from runtime_manager.msg import lamp_cmd
from runtime_manager.msg import traffic_light
from runtime_manager.msg import adjust_xy

SCHED_OTHER = 0
SCHED_FIFO = 1
SCHED_RR = 2
PROC_MANAGER_SOCK="/tmp/autoware_proc_manager"

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)
		self.all_procs = []
		self.all_procs_nodes = {}
		self.all_cmd_dics = []
		self.stop_dic = {}
		self.load_dic = self.load_yaml('param.yaml', def_ret={})
		self.config_dic = {}
		self.selector = {}
		self.Bind(wx.EVT_CLOSE, self.OnClose)
		self.params = []
		self.all_tabs = []
		self.all_th_infs = []
		self.log_que = Queue.Queue()
		self.log_que_stdout = Queue.Queue()
		self.log_que_stderr = Queue.Queue()
		self.log_que_show = Queue.Queue()

		#
		# ros
		#
		rospy.init_node('runime_manager', anonymous=True)
		rospy.Subscriber('to_rtmgr', std_msgs.msg.String, self.RosCb)
		self.pub = rospy.Publisher('from_rtmgr', std_msgs.msg.String, queue_size=10)

		#
		# for Quick Start tab
		#
		tab = self.tab_qs
		self.all_tabs.append(tab)

		self.qs_cmd = {}
		self.all_cmd_dics.append(self.qs_cmd)
		self.qs_dic = self.load_yaml('qs.yaml')

		self.add_params(self.qs_dic.get('params', []))

		self.setup_buttons(self.qs_dic.get('buttons', {}), self.qs_cmd)

		for nm in [ 'map', 'sensing', 'localization', 'detection', 'mission_planning', 'motion_planning' ]:
			btn = self.obj_get('button_' + nm + '_qs')
			pnl = self.obj_get('panel_' + nm + '_qs')
			self.set_param_panel(btn, pnl)

			for key in self.qs_dic.get('exec_time', {}).get(nm, {}).keys():
				(topic, msg, attr) = ( key.split('.') + [ None, None, None ] )[:3]
				msg = globals().get(msg)
				msg = msg if msg else std_msgs.msg.Float32
				attr = attr if attr else 'data'
				rospy.Subscriber(topic, msg, self.exec_time_callback, callback_args=(key, attr))

		#
		# for Setup tab
		#
		tab = self.tab_setup
		self.all_tabs.append(tab)

		setup_cmd = {}
		self.all_cmd_dics.append(setup_cmd)
		dic = self.load_yaml('setup.yaml')
		
		self.add_params(dic.get('params', []))
		self.setup_buttons(dic.get('buttons', {}), setup_cmd)
		for nm in [ 'setup_tf', 'vehicle_model' ]:
			self.set_param_panel(self.obj_get('button_' + nm), self.obj_get('panel_' + nm))

		#
		# for Map tab
		#
		tab = self.tab_map
		self.all_tabs.append(tab)

		self.map_cmd = {}
		self.all_cmd_dics.append(self.map_cmd)
		self.map_dic = self.load_yaml('map.yaml')

		self.add_params(self.map_dic.get('params', []))
		self.selector.update(self.map_dic.get('selector', {}))

		self.setup_buttons(self.map_dic.get('buttons', {}), self.map_cmd)

		for nm in [ 'point_cloud', 'vector_map', 'area_lists', 'tf', 'pcd_filter', 'pcd_binarizer' ]:
			btn = self.obj_get('button_' + nm)
			pnl = self.obj_get('panel_' + nm)
			self.set_param_panel(btn, pnl)

		self.tc_point_cloud = self.obj_to_varpanel_tc(self.button_point_cloud, 'path_pcd')
		self.tc_area_list = self.obj_to_varpanel_tc(self.button_area_lists, 'path_area_list')

		self.label_point_cloud_bar.Destroy()
		self.label_point_cloud_bar = BarLabel(tab, '  Loading...  ')
		self.label_point_cloud_bar.Enable(False)

		self.pcd_loaded = False

		#
		# for Sensing tab
		#
		tab = self.tab_sensing
		self.all_tabs.append(tab)

		self.drv_probe_cmd = {}
		self.sensing_cmd = {}
		self.all_cmd_dics.append(self.sensing_cmd)
		dic = self.load_yaml('sensing.yaml')

		self.add_params(dic.get('params', []))

		self.create_checkboxes(dic, self.panel_sensing, None, self.drv_probe_cmd, self.sensing_cmd, self.OnSensingDriver)

		self.setup_buttons(dic.get('buttons', {}), self.sensing_cmd)

		#self.timer = wx.Timer(self)
		#self.Bind(wx.EVT_TIMER, self.OnProbe, self.timer)
		#self.probe_interval = 10*1000
		#if self.checkbox_auto_probe.GetValue():
		#	self.OnProbe(None)
		#	self.timer.Start(self.probe_interval)

		self.dlg_rosbag_record = MyDialogRosbagRecord(self, cmd_dic=self.sensing_cmd)

		#
		# for Computing tab
		#
		tab = self.tab_computing
		self.all_tabs.append(tab)

		parent = self.tree_ctrl_0.GetParent()
		for i in range(2):
			self.obj_get('tree_ctrl_' + str(i)).Destroy()
		items = self.load_yaml('computing.yaml')

		self.add_params(items.get('params', []))

		self.sys_gdic = items.get('sys_gui')
		self.sys_gdic['update_func'] = self.update_func

		self.computing_cmd = {}
		self.all_cmd_dics.append(self.computing_cmd)
		for i in range(2):
			tree_ctrl = self.create_tree(parent, items['subs'][i], None, None, self.computing_cmd)
			tree_ctrl.ExpandAll()
			tree_ctrl.SetBackgroundColour(wx.NullColour)
			setattr(self, 'tree_ctrl_' + str(i), tree_ctrl)

		self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)

		self.setup_buttons(items.get('buttons', {}), self.computing_cmd)

		#
		# for Interface tab
		#
		tab = self.tab_interface
		self.all_tabs.append(tab)

		self.interface_cmd = {}
		self.all_cmd_dics.append(self.interface_cmd)
		self.interface_dic = self.load_yaml('interface.yaml')

		self.add_params(self.interface_dic.get('params', []))

		self.setup_buttons(self.interface_dic.get('buttons', {}), self.interface_cmd)
		self.setup_buttons(self.interface_dic.get('checkboxs', {}), self.interface_cmd)

		szr = wx.BoxSizer(wx.VERTICAL)
		for cc in self.interface_dic.get('control_check', []):
			pdic = {}
			prm = self.get_param(cc.get('param'))
			for var in prm['vars']:
				pdic[ var['name'] ] = var['v']
			gdic = self.gdic_get_1st(cc)
			panel = ParamPanel(self.panel_interface_cc, frame=self, pdic=pdic, gdic=gdic, prm=prm)
			szr.Add(panel, 0, wx.EXPAND)
		self.panel_interface_cc.SetSizer(szr)

		#
		# for Database tab
		#
		tab = self.tab_database
		self.all_tabs.append(tab)

		self.data_cmd = {}
		self.all_cmd_dics.append(self.data_cmd)
		dic = self.load_yaml('data.yaml')

		self.add_params(dic.get('params', []))

		parent = self.tree_ctrl_data.GetParent()
		self.tree_ctrl_data.Destroy()
		tree_ctrl = self.create_tree(parent, dic, None, None, self.data_cmd)
		tree_ctrl.ExpandAll()
		tree_ctrl.SetBackgroundColour(wx.NullColour)
		self.tree_ctrl_data = tree_ctrl

		#self.setup_config_param_pdic()

		if 'buttons' in dic:
			self.setup_buttons(dic['buttons'], self.data_cmd)

		#
		# for Simulation Tab
		#
		tab = self.tab_simulation
		self.all_tabs.append(tab)

		self.simulation_cmd = {}
		self.all_cmd_dics.append(self.simulation_cmd)
		dic = self.load_yaml('simulation.yaml')

		self.add_params(dic.get('params', []))
		self.selector.update(dic.get('selector', {}))

		self.setup_buttons(dic.get('buttons'), self.simulation_cmd)

		btn = self.button_play_rosbag_play
		pnl = self.panel_rosbag_play
		self.set_param_panel(btn, pnl)

		# setup for rosbag info
		gdic = self.obj_to_gdic(btn, {})
		name = 'file'
		if name not in gdic:
			gdic[ name ] = {}
		gdic_v = gdic.get(name, {})
		gdic_v['update_hook'] = self.rosbag_info_hook

		tc = self.obj_to_varpanel_tc(btn, 'file')
		if tc:
			self.rosbag_info_hook( tc.GetValue() )


		#vp = self.obj_to_varpanel(btn, 'sim_time')
		#self.checkbox_sim_time = vp.obj

		#try:
		#	cmd = ['rosparam', 'get', '/use_sim_time']
		#	if subprocess.check_output(cmd, stderr=open(os.devnull, 'wb')).strip() == 'true':
		#		self.checkbox_sim_time.SetValue(True)
		#except subprocess.CalledProcessError:
		#	pass

		self.label_rosbag_play_bar.Destroy()
		self.label_rosbag_play_bar = BarLabel(tab, '  Playing...  ')
		self.label_rosbag_play_bar.Enable(False)

		#
		# for Status tab
		#
		tab = self.tab_status
		self.all_tabs.append(tab)

		self.status_cmd = {}
		self.all_cmd_dics.append(self.status_cmd)
		self.status_dic = self.load_yaml('status.yaml')

		self.add_params(self.status_dic.get('params', []))

		self.setup_buttons(self.status_dic.get('buttons', {}), self.status_cmd)

		font = wx.Font(10, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
		self.label_top_cmd.SetFont(font)

		#
		# for All
		#
		self.nodes_dic = self.nodes_dic_get()

		self.bitmap_logo.Destroy()
		bm = scaled_bitmap(wx.Bitmap(rtmgr_src_dir() + 'autoware_logo_1.png'), 0.2)
		self.bitmap_logo = wx.StaticBitmap(self, wx.ID_ANY, bm)

		rtmgr.MyFrame.__do_layout(self)

		self.alias_grps = [
			[ self.button_rviz_qs, self.button_rviz_map, self.button_rviz_sensing, self.button_rviz_computing,
			  self.button_rviz_interface, self.button_rviz_database, self.button_rviz_simulation,
			  self.button_rviz_status, self.button_rviz_topics, ],
			[ self.button_rqt_qs, self.button_rqt_map, self.button_rqt_sensing, self.button_rqt_computing,
			  self.button_rqt_interface, self.button_rqt_database, self.button_rqt_simulation,
			  self.button_rqt_status, self.button_rqt_topics, ],
			[ self.button_android_tablet_qs, self.button_android_tablet_interface, ],
			[ self.button_oculus_rift_qs, self.button_oculus_rift_interface, ],
			[ self.button_vehicle_gateway_qs, self.button_vehicle_gateway_interface, ],
			[ self.button_auto_pilot_qs, self.button_auto_pilot_interface,],
		]
		for grp in self.alias_grps:
			wx.CallAfter(self.alias_sync, get_top(grp))

		# Topics tab (need, after layout for sizer)
		self.topics_dic = self.load_yaml('topics.yaml')
		self.topics_list = []
		self.topics_echo_curr_topic = None
		self.topics_echo_proc = None
		self.topics_echo_thinf = None

		self.topics_echo_que = Queue.Queue()
		self.topics_echo_sum = 0
		thinf = th_start(self.topics_echo_show_th)
		self.all_th_infs.append(thinf)

		self.refresh_topics_list()

		# waypoint
		self.route_cmd_waypoint = [ Waypoint(0,0), Waypoint(0,0) ]
		rospy.Subscriber('route_cmd', route_cmd, self.route_cmd_callback)

		# topic /xxx_stat
		self.stat_dic = {}
		for k in [ 'gnss', 'pmap', 'vmap', 'lf' ]:
			self.stat_dic[k] = False
			name = k + '_stat'
			rospy.Subscriber(name, std_msgs.msg.Bool, self.stat_callback, callback_args=k)

		# top command thread setup
		toprc = os.path.expanduser('~/.toprc')
		backup = os.path.expanduser('~/.toprc-autoware-backup')
		self.toprc_setup(toprc, backup)

		cpu_ibls = [ InfoBarLabel(self, 'CPU'+str(i)) for i in range(psutil.NUM_CPUS) ]
		sz = sizer_wrap(cpu_ibls, wx.HORIZONTAL, 1, wx.EXPAND, 0)
		self.sizer_cpuinfo.Add(sz, 8, wx.ALL | wx.EXPAND, 4)

		self.lb_top5 = []
		for i in range(5):
			lb = wx.StaticText(self, wx.ID_ANY, '')
			change_font_point_by_rate(lb, 0.75)
			self.lb_top5.append(lb)
		line = wx.StaticLine(self, wx.ID_ANY)
		ibl = InfoBarLabel(self, 'Memory', bar_orient=wx.HORIZONTAL)
		szr = sizer_wrap(self.lb_top5 + [ line, ibl ], flag=wx.EXPAND | wx.FIXED_MINSIZE)
		self.sizer_cpuinfo.Add(szr, 2, wx.ALL | wx.EXPAND, 4)

		th_arg = { 'setting':self.status_dic.get('top_cmd_setting', {}),
			   'cpu_ibls':cpu_ibls, 'mem_ibl':ibl, 
			   'toprc':toprc, 'backup':backup }
		thinf = th_start(self.top_cmd_th, th_arg)
		self.all_th_infs.append(thinf)

		# ps command thread
		#thinf = th_start(self.ps_cmd_th, { 'interval':5 })
		#self.all_th_infs.append(thinf)

		# logout thread
		interval = self.status_dic.get('gui_update_interval_ms', 100) * 0.001
		tc = self.text_ctrl_stdout

		thinf = th_start(self.logout_th, { 'que':self.log_que_stdout, 'interval':interval, 'tc':tc } )
		self.all_th_infs.append(thinf)
		thinf = th_start(self.logout_th, { 'que':self.log_que_stderr, 'interval':interval, 'tc':tc } )
		self.all_th_infs.append(thinf)
		thinf = th_start(self.logout_th, { 'que':self.log_que, 'interval':interval, 'tc':tc } )
		self.all_th_infs.append(thinf)

		if interval > 0:
			thinf = th_start(self.logshow_th, { 'que':self.log_que_show , 'interval':interval , 'tc':tc })
			self.all_th_infs.append(thinf)
		else:
			self.checkbox_stdout.Enable(False)
			tc.Enable(False)

		# mkdir
		paths = [ os.environ['HOME'] + '/.autoware/data/tf',
			  os.environ['HOME'] + '/.autoware/data/map/pointcloud_map',
			  os.environ['HOME'] + '/.autoware/data/map/vector_map' ]
		for path in paths:
			if not os.path.exists(path):
				subprocess.call([ 'mkdir', '-p', path ])

		# icon
		bm = scaled_bitmap(wx.Bitmap(rtmgr_src_dir() + 'autoware_logo_2_white.png'), 0.5)
		icon = wx.EmptyIcon()
		icon.CopyFromBitmap(bm)
		self.SetIcon(icon)

	def __do_layout(self):
		pass

	def OnClose(self, event):
		self.kill_all()

		save_dic = {}
		for (name, pdic) in self.load_dic.items():
			if pdic and pdic != {}:
				prm = next( (cfg.get('param') for cfg in self.config_dic.values() if cfg.get('name') == name), {})
				no_saves = prm.get('no_save_vars', [])
				pdic = pdic.copy()
				for k in pdic.keys():
					if k in no_saves:
						del pdic[k]
				save_dic[name] = pdic
		if save_dic != {}:
			dir = rtmgr_src_dir()
			print('saving param.yaml')
			f = open(dir + 'param.yaml', 'w')
			s = yaml.dump(save_dic, default_flow_style=False)
			#print 'save\n', s # for debug
			f.write(s)
			f.close()

		shutdown_proc_manager()

		shutdown_sh = self.get_autoware_dir() + '/ros/shutdown'
		if os.path.exists(shutdown_sh):
			os.system(shutdown_sh)

		for thinf in self.all_th_infs:
			th_end(thinf)

		self.Destroy()

	def RosCb(self, data):
		print('recv topic msg : ' + data.data)

		r = rospy.Rate(10)
		rospy.is_shutdown()
		r.sleep()
		self.pub.publish(data.data)
		r.sleep()

	def try_setup_obj_stop(self, obj, dic):
		if 'stop' in dic:
			self.stop_dic[ obj ] = dic.get('stop')

	def setup_buttons(self, d, run_dic):
		for (k,d2) in d.items():
			pfs = [ 'button_', 'button_launch_', 'checkbox_' ]
			obj = next( (self.obj_get(pf+k) for pf in pfs if self.obj_get(pf+k)), None)
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
			self.try_setup_obj_stop(obj, d2);
			if 'param' in d2:
				pdic = self.load_dic_pdic_setup(k, d2)
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

	#def OnDrive(self, event):
	#	obj = event.GetEventObject()
	#	v = obj.GetValue()
	#	pub = rospy.Publisher('mode_cmd', mode_cmd, queue_size=10)
	#	pub.publish(mode_cmd(mode=v))

	def OnTextRoute(self, event):
		pass

	#def OnLoadMap(self, event):
	#	obj = event.GetEventObject()
	#	self.OnSelector_obj(obj)

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

	def OnLamp(self, event):
		pub = rospy.Publisher('lamp_cmd', lamp_cmd, queue_size=10)
		msg = lamp_cmd()
		msg.l = self.button_statchk_lamp_l.GetValue()
		msg.r = self.button_statchk_lamp_r.GetValue()
		pub.publish(msg)

	def OnIndi(self, event):
		pub = rospy.Publisher('indicator_cmd', indicator_cmd, queue_size=10)
		msg = indicator_cmd()
		msg.l = self.button_statchk_indi_l.GetValue()
		msg.r = self.button_statchk_indi_r.GetValue()
		pub.publish(msg)

	def OnAutoPilot(self, event):
		obj = event.GetEventObject()
		self.alias_sync(obj)
		v = obj.GetValue()
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
		qs_nms = [ 'map', 'sensing', 'localization', 'detection', 'mission_planning', 'motion_planning' ]
		exec_time = self.qs_dic.get('exec_time', {})

		gdic = self.obj_to_gdic(obj, {})
		msg = std_msgs.msg.Bool(False)
		for k in gdic.get('stat_topic', []):
			# exec_time off
			if next( (dic for dic in exec_time.values() if k in dic), None):
				self.exec_time_callback(std_msgs.msg.Float32(0), (k, 'data'))
			else:
				self.stat_callback(msg, k)

		# Quick Start tab, exec_time off
		obj_nm = self.name_get(obj)
		nm = next( (nm for nm in qs_nms if 'button_' + nm + '_qs' == obj_nm), None)
		for key in exec_time.get(nm, {}):
			self.exec_time_callback(std_msgs.msg.Float32(0), (key, 'data'))

	def route_cmd_callback(self, data):
		self.route_cmd_waypoint = data.point

	def stat_callback(self, msg, k):
		if k == 'pmap' and msg.data and not self.pcd_loaded:
			return
		self.stat_dic[k] = msg.data
		if k == 'pmap':
			v = self.stat_dic.get(k)
			wx.CallAfter(self.label_point_cloud.SetLabel, 'OK' if v else '')
		if k in [ 'pmap', 'vmap' ]:
			v = self.stat_dic.get('pmap') and self.stat_dic.get('vmap')
			wx.CallAfter(self.label_map_qs.SetLabel, 'OK' if v else '')

	def exec_time_callback(self, msg, (key, attr)):
		msec = int(getattr(msg, attr, 0))
		exec_time = self.qs_dic.get('exec_time', {})
		(nm, dic) = next( ( (nm, dic) for (nm, dic) in exec_time.items() if key in dic), None)
		dic[ key ] = msec
		lb = self.obj_get('label_' + nm + '_qs')
		if lb:
			sum = reduce( lambda a,b:a+(b if b else 0), dic.values(), 0 )
			wx.CallAfter(lb.SetLabel, str(sum)+' ms' if sum > 0 else '')

		# update Status tab
		lb = ''
		for nm in [ 'map', 'sensing', 'localization', 'detection', 'mission_planning', 'motion_planning' ]:
			dic = exec_time.get(nm, {})
			sum = reduce( lambda a,b:a+(b if b else 0), dic.values(), 0 )
			if sum > 0:
				s = nm + ' : ' + str(sum) + ' ms'
				lb += s + '\n'
		wx.CallAfter(self.label_node_time.SetLabel, lb)
		wx.CallAfter(self.label_node_time.GetParent().FitInside)

	#
	# Setup tab
	#
	def OnSetupLocalizer(self, event):
		obj = self.button_setup_tf
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		self.update_func(pdic, gdic, prm)

	#
	# Computing Tab
	#
	def OnTreeChecked(self, event):
		self.OnChecked_obj(event.GetItem())

	def OnChecked_obj(self, obj):
		self.OnLaunchKill_obj(obj)

	#def OnTreeHyperlinked(self, event):
	#	self.OnHyperlinked_obj(event.GetItem())

	def OnHyperlinked(self, event):
		self.OnHyperlinked_obj(event.GetEventObject())

	def OnHyperlinked_obj(self, obj):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		if pdic is None or prm is None:
			return
		dic_list_push(gdic, 'dialog_type', 'config')
		klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
		dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
		dlg.ShowModal()
		dic_list_pop(gdic, 'dialog_type')

	def obj_to_add_args(self, obj, msg_box=True):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		if pdic is None or prm is None:
			return None

		if 'open_dialog' in gdic.get('flags', []) and msg_box:
			dic_list_push(gdic, 'dialog_type', 'open')
			klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
			dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
			dlg_ret = dlg.ShowModal()
			dic_list_pop(gdic, 'dialog_type')
			if dlg_ret != 0:
				return False			

		self.update_func(pdic, gdic, prm)
		s = ''

		vars = []
		for var in prm.get('vars'):
			cmd_param = var.get('cmd_param')
			if cmd_param:
				vars.append(var)

		for var in vars[:]: # copy
			cmd_param = var.get('cmd_param')
			if cmd_param.get('tail'):
				vars.append( vars.pop(0) )

		for var in vars[:]: # copy
			name = var.get('name')
			flags = gdic.get(name, {}).get('flags', [])
			if 'hide' in flags or 'disable' in flags:
				vars.remove(var)

		for var in vars:
			cmd_param = var.get('cmd_param')
			name = var.get('name')
			v = pdic.get(name)
			if (v is None or v == '') and 'default' in cmd_param:
				v = cmd_param.get('default')
			if cmd_param.get('must') and (v is None or v == ''):
				print 'cmd_param', name, 'is required'
				if msg_box:
					wx.MessageBox('cmd_param ' + name + ' is required')
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
					str_v = path_expand_cmd(str_v)
					str_v = os.path.expandvars(os.path.expanduser(str_v))

					relpath_from = var.get('relpath_from')
					if relpath_from:
						relpath_from = path_expand_cmd(relpath_from)
						relpath_from = os.path.expandvars(os.path.expanduser(relpath_from))
						str_v = os.path.relpath(str_v, relpath_from)
				add += delim + str_v
			if add != '':
				s += add + ' '
		return s.strip(' ').split(' ') if s != '' else None

	def obj_to_pdic_gdic_prm(self, obj, sys=False):
		info = self.config_dic.get(obj)
		if info is None:
			sys_prm = self.get_param('sys')
			prm_chk = lambda prm : prm is sys_prm if sys else prm is not sys_prm
			info = next( ( v for v in self.config_dic.values() if v.get('obj') is obj and prm_chk(v.get('param')) ), None)
			if info is None:
				return (None, None, None)
		pdic = info.get('pdic')
		prm = info.get('param')
		gdic = info.get('gdic')
		return (pdic, gdic, prm)

	def obj_to_gdic(self, obj, def_ret=None):
		(_, gdic, _) = self.obj_to_pdic_gdic_prm(obj) if obj else (None, None, None)
		return gdic if gdic else def_ret

	def cfg_prm_to_obj(self, arg_dic):
		return next( ( d.get('obj') for d in self.config_dic.values() \
			if all( [ d.get(k) == v for (k,v) in arg_dic.items() ] ) ), None)

	def update_func(self, pdic, gdic, prm):
		pdic_empty = (pdic == {})
		for var in prm.get('vars', []):
			name = var.get('name')
			gdic_v = gdic.get(name, {})
			func = gdic_v.get('func')
			if func is None and not pdic_empty:
				continue
			v = var.get('v')
			if func is not None:
				v = eval(func) if type(func) is str else func()
			pdic[ name ] = v

			hook = gdic_v.get('update_hook')
			if hook:
				hook(v)

		if 'pub' in prm:
			self.publish_param_topic(pdic, prm)
		self.rosparam_set(pdic, prm)
		self.update_depend_enable(pdic, gdic, prm)

		sys_gdic = getattr(self, 'sys_gdic', None)
		obj = self.cfg_prm_to_obj( { 'pdic':pdic , 'gdic':sys_gdic , 'param':prm } )
		self.update_proc_cpu(obj, pdic, prm)

	def update_proc_cpu(self, obj, pdic=None, prm=None):
		if obj is None or not obj.GetValue():
			return
		(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if proc is None:
			return
		if pdic is None or prm is None:
			(pdic, _, prm) = self.obj_to_pdic_gdic_prm(obj, sys=True)

		cpu_chks = self.param_value_get(pdic, prm, 'cpu_chks')
		cpu_chks = cpu_chks if cpu_chks else [ True for i in range(psutil.NUM_CPUS) ]
		cpus = [ i for i in range(psutil.NUM_CPUS) if cpu_chks[i] ]
		nice = self.param_value_get(pdic, prm, 'nice', 0)

		d = { 'OTHER':SCHED_OTHER, 'FIFO':SCHED_FIFO, 'RR':SCHED_RR }
		policy = SCHED_OTHER
		priority = 0
		if self.param_value_get(pdic, prm, 'real_time', False):
			policy = d.get(self.param_value_get(pdic, prm, 'policy', 'FIFO'), SCHED_FIFO)
			priority = self.param_value_get(pdic, prm, 'prio', 0)

		procs = [ proc ] + proc.get_children(recursive=True)
		for proc in procs:
			print 'pid={}'.format(proc.pid)
			if proc.get_nice() != nice:
				print 'nice {} -> {}'.format(proc.get_nice(), nice)
				if set_process_nice(proc, nice) is False:
					print 'Err set_process_nice()'
			if proc.get_cpu_affinity() != cpus:
				print 'cpus {} -> {}'.format(proc.get_cpu_affinity(), cpus)
				if set_process_cpu_affinity(proc, cpus) is False:
					print 'Err set_process_cpu_affinity()'

			policy_str = next( (k for (k,v) in d.items() if v == policy), '?')
			print 'sched policy={} prio={}'.format(policy_str, priority)
			if set_scheduling_policy(proc, policy, priority) is False:
				print 'Err scheduling_policy()'

	def param_value_get(self, pdic, prm, name, def_ret=None):
		def_ret = self.param_default_value_get(prm, name, def_ret)
		return pdic.get(name, def_ret) if pdic else def_ret

	def param_default_value_get(self, prm, name, def_ret=None):
		return next( (var.get('v') for var in prm.get('vars') if var.get('name') == name ), def_ret) \
			if prm else def_ret

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
			cvdic = { 'True':'true', 'False':'false' }
			if v in cvdic:
				v = cvdic.get(v)
			nm = rosparam
			nm = ('/' if len(nm) > 0 and nm[0] != '/' else '') + nm
			exist = nm in rosparams
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
	#def create_viewer_btns(self, parent, sizer, lst):
	#	for dic in lst:
	#		lb = dic.get('label')
	#		prop = 0
	#		flag = wx.ALL | wx.EXPAND
	#		border = 4
	#		if 'subs' in dic:
	#			if lb:
	#				obj = static_box_sizer(parent, lb)
	#			else:
	#				obj = wx.BoxSizer(wx.VERTICAL)
	#			self.create_viewer_btns(parent, obj, dic['subs'])
	#		else:
	#			obj = wx.ToggleButton(parent, wx.ID_ANY, lb)
	#			self.Bind(wx.EVT_TOGGLEBUTTON, self.OnViewer, obj)
	#			self.viewer_cmd[obj] = (dic['cmd'], None)
	#
	#		if sizer is self.sizer_viewer:
	#			prop = 1
	#			flag = wx.ALL | wx.ALIGN_CENTER_VERTICAL
	#		sizer.Add(obj, prop, flag, border)

	#def OnViewer(self, event):
	#	self.launch_kill_proc(event.GetEventObject(), self.viewer_cmd)

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.OnChecked_obj(event.GetEventObject())

	#def OnAutoProbe(self, event):
	#	if event.GetEventObject().GetValue():
	#		self.OnProbe(None)
	#		self.timer.Start(self.probe_interval)
	#	else:
	#		self.timer.Stop()

	#def OnProbe(self, event):
	#	#print('probe') # for debug
	#	items = self.drv_probe_cmd.items()
	#	for (obj, (cmd, bak_res)) in items:
	#		res = (os.system(cmd) == 0) if cmd else False
	#		if res == bak_res:
	#			continue
	#		self.drv_probe_cmd[obj] = (cmd, res)
	#		cfg_obj = self.get_cfg_obj(obj)
	#		en = obj.IsShown()
	#		if res and not en:
	#			obj.Show()
	#			if cfg_obj:
	#				cfg_obj.Show()
	#			continue
	#		if not res and en:
	#			v = obj.GetValue()
	#			if v:
	#				obj.SetValue(False)	
	#				self.launch_kill_proc(obj, self.sensing_cmd)
	#			obj.Hide()
	#			if cfg_obj:
	#				cfg_obj.Hide()

	def OnRosbagRecord(self, event):
		self.dlg_rosbag_record.Show()
		obj = event.GetEventObject()
		set_val(obj, False)

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
			self.try_setup_obj_stop(obj, dic);
			if 'param' in dic:
				obj = self.add_config_link(dic, panel, obj)
		if sizer:
			sizer.Add(obj, 0, wx.EXPAND | bdr_flg, 4)
		else:
			panel.SetSizer(obj)

	def add_config_link(self, dic, panel, obj):
		cfg_obj = wx.HyperlinkCtrl(panel, wx.ID_ANY, '[config]', '')
		fix_link_color(cfg_obj)
		self.Bind(wx.EVT_HYPERLINK, self.OnConfig, cfg_obj)
		add_objs = (obj, wx.StaticText(panel, wx.ID_ANY, '  '), cfg_obj)
		hszr = sizer_wrap(add_objs, wx.HORIZONTAL)
		name = dic['name']
		pdic = self.load_dic_pdic_setup(name, dic)
		gdic = self.gdic_get_1st(dic)
		prm = self.get_param(dic.get('param'))
		self.add_cfg_info(cfg_obj, obj, name, pdic, gdic, True, prm)
		return hszr

	#
	# Simulation Tab
	#
	def rosbag_info_hook(self, v):
		if not v:
			return
		err = subprocess.STDOUT
		s = subprocess.check_output([ 'rosbag', 'info', v ], stderr=err).strip()
		self.label_rosbag_info.SetLabel(s)
		self.label_rosbag_info.GetParent().FitInside()

	#def OnSimulation(self, event):
	#	self.OnChecked_obj(event.GetEventObject())

	#def OnSimTime(self, event):
	#	obj = event.GetEventObject()
	#	self.alias_sync(obj)
	#	obj = self.alias_grp_top_obj(obj)
	#	cmd_dic = self.simulation_cmd
	#	(cmd, proc) = cmd_dic.get(obj, (None, None))
	#	if cmd and type(cmd) is dict:
	#		cmd = cmd.get(obj.GetValue())
	#	if cmd:
	#		print(cmd)
	#		os.system(cmd)

	#def OnLaunchPmap(self, event):
	#	self.OnSelector_obj(self.button_launch_pmap)

	#def OnKillPmap(self, event):
	#	self.OnSelector_obj(self.button_kill_pmap)

	#def OnPointMapUpdate(self, event):
	#	sdic = self.selector.get('pmap', {})
	#	if sdic.get('launched'):
	#		self.OnKillPmap(None)
	#		self.OnLaunchPmap(None)

	#
	# Data Tab
	#

	#
	# Stauts tab
	#

	def info_col(self, v, v_yellow, v_red, col_normal, col_red):
		if v < v_yellow:
			return col_normal
		if v < v_red:		
			(nr,ng,nb) = col_normal
			(rr,rg,rb) = col_red
			return ( (nr+rr)/2, (ng+rg)/2, (nb+rb)/2 )
		return col_red

	def mem_kb_info(self):
		lst = subprocess.check_output(['free']).strip().split('\n')[2].split()[2:4]
		used = int(lst[0])
		free = int(lst[1])
		return (used + free, used)

	def toprc_create(self):
		(child_pid, fd) = pty.fork()
		if child_pid == 0: # child
			os.execvp('top', ['top'])
		else: #parent
			sec = 0.2
			for s in ['1', 'c', 'W', 'q']:
				time.sleep(sec)
				os.write(fd, s)

	def toprc_setup(self, toprc, backup):
		if os.path.exists(toprc):
			os.rename(toprc, backup)
		self.toprc_create()

	def toprc_restore(self, toprc, backup):
		os.remove(toprc)
		if os.path.exists(backup):
			os.rename(backup, toprc)

	# top command thread
	def top_cmd_th(self, ev, setting, cpu_ibls, mem_ibl, toprc, backup):
		interval = setting.get('interval', 3)
		alert_level = setting.get('alert_level', {})
		rate_per_cpu = alert_level.get('rate_per_cpu', 80)
		rate_per_cpu_yellow = alert_level.get('rate_per_cpu_yellow', 80)
		rate_cpu = alert_level.get('rate_cpu', 80)
		rate_mem = alert_level.get('rate_mem', 80)
		rate_mem_yellow = alert_level.get('rate_mem_yellow', 80)

		for ibl in cpu_ibls:
			ibl.lmt_bar_prg = rate_per_cpu
		mem_ibl.lmt_bar_prg = rate_mem

		alerted = False
		cpu_n = psutil.NUM_CPUS

		while not ev.wait(interval):
			s = subprocess.check_output(['sh', '-c', 'env COLUMNS=512 top -b -n 2 -d 0.1']).strip()
			i = s.rfind('\ntop -') + 1
			s = s[i:]
			wx.CallAfter(self.label_top_cmd.SetLabel, s)
			wx.CallAfter(self.label_top_cmd.GetParent().FitInside)

			k = '%Cpu'
			fv_sum = 0
			i = 0
			for t in s.split('\n'):
				if t[:len(k)] != k:
					continue
				lst = t[1:].split()
				v = lst[1] if lst[1] != ':' else lst[2]
				if v[0] == ':':
					v = v[1:]
				fv = float(v)
				col = self.info_col(fv, rate_per_cpu_yellow, rate_per_cpu, (64,64,64), (200,0,0))

				if i < cpu_n:
					ibl = cpu_ibls[i]
					wx.CallAfter(ibl.lb_set, v+'%', col)
					wx.CallAfter(ibl.bar_set, int(fv))
					fv_sum += fv
				i += 1

			k = 'KiB Mem:'
			(total, used) = self.mem_kb_info()
			rate = 100 * used / total

			for u in [ 'KB', 'MB', 'GB', 'TB' ]:
				if total <= 10 * 1024 or used <= 10:
					break
				total /= 1024
				used /= 1024

			col = self.info_col(rate, rate_mem_yellow, rate_mem, (64,64,64), (200,0,0))
			tx = str(used) + u + '/' + str(total) + u + '(' + str(rate) + '%)'

			wx.CallAfter(mem_ibl.lb_set, tx, col)
			wx.CallAfter(mem_ibl.bar_set, rate)

			is_alert = (fv_sum >= rate_cpu * cpu_n) or rate >= rate_mem

			# --> for test
			if os.path.exists('/tmp/alert_test_on'):
				is_alert = True
			if os.path.exists('/tmp/alert_test_off'):
				is_alert = False
			# <-- for test

			if is_alert and not alerted:
				thinf = th_start(self.alert_th, {'bgcol':(200,50,50)})
				alerted = True
			if not is_alert and alerted:
				th_end(thinf)
				alerted = False

			# top5
			i = s.find('\n\n') + 2
			lst = s[i:].split('\n')
			hd = lst[0]
			top5 = lst[1:1+5]

			i = hd.rfind('COMMAND')
			cmds = [ line[i:].split(' ')[0] for line in top5 ]

			i = hd.find('%CPU')
			loads = [ line[i-1:].strip().split(' ')[0] for line in top5 ]
			
			for (lb, cmd, load) in zip(self.lb_top5, cmds, loads):
				col = self.info_col(float(load), rate_per_cpu_yellow, rate_per_cpu, (64,64,64), (200,0,0))
				wx.CallAfter(lb.SetForegroundColour, col)
				wx.CallAfter(lb.SetLabel, cmd + ' (' + load + ' %CPU)')

		self.toprc_restore(toprc, backup)

	def alert_th(self, bgcol, ev):
		wx.CallAfter(self.RequestUserAttention)
		c = bgcol
		o = wx.NullColour
		while not ev.wait(0.5):
			for col in [ c, o, c, o, c, o ]:
				wx.CallAfter(self.set_bg_all_tabs, col)
				time.sleep(0.05)

	# ps command thread
	#def ps_cmd_th(self, ev, interval):
	#	nodes = reduce(lambda a,b:a+b, [[]] + self.nodes_dic.values())
	#	while not ev.wait(interval):
	#		lb = ''
	#		for s in subprocess.check_output(['ps', '-eo' 'etime,args']).strip().split('\n')[1:]:
	#			if ('/' + s.split('/')[-1]) in nodes:
	#				lb += s + '\n'
	#		wx.CallAfter(self.label_node_time.SetLabel, lb)
	#		wx.CallAfter(self.label_node_time.GetParent().FitInside)

	def log_th(self, file, que, ev):
		while not ev.wait(0):
			s = file.readline()
			if not s:
				break
			que.put(s)

	def logout_th(self, que, interval, tc, ev):
		if que == self.log_que_stdout or que == self.log_que_stderr:
			while not ev.wait(0):
				try:
					s = que.get(timeout=1)
				except Queue.Empty:
					continue
				self.log_que.put(s)

				if interval <= 0:
					continue

				ckbox = self.checkbox_stdout if que == self.log_que_stdout else self.checkbox_stderr
				if ckbox.GetValue():
					self.log_que_show.put( cut_esc(s) )

		else: # == self.log_que
			f = None
			path = self.status_dic.get('log_path')
			is_syslog = (path == 'syslog')

			if is_syslog:
				ident = sys.argv[0].split('/')[-1]
				syslog.openlog(ident, syslog.LOG_PID | syslog.LOG_CONS)
			elif path:
				path = os.path.expandvars(os.path.expanduser(path))
				f = open(path, 'a') if path else None

			while not ev.wait(0):
				try:
					s = que.get(timeout=1)
				except Queue.Empty:
					continue
				print s.strip()
				sys.stdout.flush()

				s = cut_esc(s)
				if is_syslog:
					syslog.syslog(s)
				elif f:
					f.write(s)
					f.flush()
			if is_syslog:
				syslog.closelog()
			if f:
				f.close()

	def logshow_th(self, que, interval, tc, ev):
		while not ev.wait(interval):
			try:
				s = que.get(timeout=1)
			except Queue.Empty:
				continue
			wx.CallAfter(append_tc_limit, tc, s)

			# que clear
			if self.checkbox_stdout.GetValue() is False and \
			   self.checkbox_stderr.GetValue() is False and \
			   que.qsize() > 0:
				que_clear(que)
				wx.CallAfter(tc.Clear)

	#
	# for Topics tab
	#
	def OnRefreshTopics(self, event):
		self.refresh_topics_list()

	def refresh_topics_list(self):
		lst = subprocess.check_output([ 'rostopic', 'list' ]).strip().split('\n')
		panel = self.panel_topics_list
		szr = self.sizer_topics_list
		for obj in self.topics_list:
			szr.Remove(obj)
			obj.Destroy()
		self.topics_list = []
		for topic in lst:
			obj = wx.HyperlinkCtrl(panel, wx.ID_ANY, topic, '')
			self.Bind(wx.EVT_HYPERLINK, self.OnTopicLink, obj)
			szr.Add(obj, 0, wx.LEFT, 4)
			fix_link_color(obj)
			self.topics_list.append(obj)
		szr.Layout()
		panel.SetVirtualSize(szr.GetMinSize())

		# info clear
		lb = self.label_topics_info
		lb.SetLabel('')
		
		# echo clear
		self.topics_proc_th_end()

		# wait que clear
		while self.topics_echo_que.qsize() > 0:
			time.sleep(0.1)

		tc = self.text_ctrl_topics_echo
		tc.Enable(False)
		wx.CallAfter(tc.Clear)
		wx.CallAfter(tc.Enable, True)
		self.topics_echo_sum = 0
		self.topic_echo_curr_topic = None

	def OnEcho(self, event):
		if self.checkbox_topics_echo.GetValue() and self.topic_echo_curr_topic:
			self.topics_proc_th_start(self.topic_echo_curr_topic)
		else:
			self.topics_proc_th_end()

	def OnTopicLink(self, event):
		obj = event.GetEventObject()
		topic = obj.GetLabel()
		self.topic_echo_curr_topic = topic

		# info
		info = subprocess.check_output([ 'rostopic', 'info', topic ]).strip()
		lb = self.label_topics_info
		lb.SetLabel(info)
		lb.GetParent().FitInside()

		# echo
		self.topics_proc_th_end()
		if self.checkbox_topics_echo.GetValue():
			self.topics_proc_th_start(topic)

	def topics_proc_th_start(self, topic):
		out = subprocess.PIPE
		err = subprocess.STDOUT
		self.topics_echo_proc = psutil.Popen([ 'rostopic', 'echo', topic ], stdout=out, stderr=err)

		self.topics_echo_thinf = th_start(self.topics_echo_th)
		
	def topics_proc_th_end(self):
		thinf = self.topics_echo_thinf
		if thinf:
			th_end(thinf)
			self.topics_echo_thinf = None

		proc = self.topics_echo_proc
		if proc:
			terminate_children(proc)
			terminate(proc)
			#proc.wait()
			self.topics_echo_proc = None

	def topics_echo_th(self, ev):
		if not self.topics_echo_proc:
			return
		file = self.topics_echo_proc.stdout
		fl = fcntl.fcntl(file.fileno(), fcntl.F_GETFL)
		fcntl.fcntl(file.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)

		while not ev.wait(0):
			try:
				s = file.read(1)
			except:
				continue
			if not s:
				break
			if self.checkbox_topics_echo.GetValue():
				self.topics_echo_que.put(s)

		que_clear(self.topics_echo_que)

	def topics_echo_show_th(self, ev):
		que = self.topics_echo_que
		interval = self.topics_dic.get('gui_update_interval_ms', 100) * 0.001
		chars_limit = self.topics_dic.get('gui_chars_limit', 10000)
		tc = self.text_ctrl_topics_echo
		while not ev.wait(interval):
			qsz = que.qsize()
			if qsz <= 0:
				continue
			if qsz > chars_limit:
				over = qsz - chars_limit
				for i in range(over):
					try:
						que.get(timeout=1)
					except Queue.Empty:
						break
				qsz = chars_limit
			arr = []
			for i in range(qsz):
				try:
					s = que.get(timeout=1)
				except Queue.Empty:
					s = ''
				arr.append(s)
			s = ''.join(arr)

			self.topics_echo_sum += len(s)
			rm_chars = 0
			if self.topics_echo_sum > chars_limit:
				rm_chars = self.topics_echo_sum - chars_limit
				self.topics_echo_sum = chars_limit

			if self.checkbox_topics_echo.GetValue():
				wx.CallAfter(append_tc_limit, tc, s, rm_chars)

	#
	# Common Utils
	#
	def OnSelector(self, event):
		self.OnSelector_obj(event.GetEventObject())

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
		sizer_wrap((panel,), wx.VERTICAL, 0, wx.EXPAND, 0, parent)
		k = 'ext_toggle_enables'
		gdic[ k ] = gdic.get(k, []) + [ panel ]

	def obj_to_varpanel(self, obj, var_name):
		gdic = self.obj_to_gdic(obj, {})
		return gdic.get(var_name, {}).get('var')

	def obj_to_varpanel_tc(self, obj, var_name):
		vp = self.obj_to_varpanel(obj, var_name)
		return vp.tc if vp and vp.tc else None

	def OnConfig(self, event):
		self.OnHyperlinked_obj(event.GetEventObject())

	def add_params(self, params):
		for prm in params:
			if 'topic' in prm and 'msg' in prm:
				klass_msg = globals()[ prm['msg'] ]
				prm['pub'] = rospy.Publisher(prm['topic'], klass_msg, latch=True, queue_size=10)
		self.params += params

	def gdic_get_1st(self, dic):
		gdic = dic.get('gui', {})
		gdic['update_func'] = self.update_func
		return gdic

	def get_cfg_obj(self, obj):
		return next( (k for (k,v) in self.config_dic.items() if v['obj'] is obj), None)

	def add_cfg_info(self, cfg_obj, obj, name, pdic, gdic, run_disable, prm):
		self.config_dic[ cfg_obj ] = { 'obj':obj , 'name':name , 'pdic':pdic , 'gdic':gdic, 
					       'run_disable':run_disable , 'param':prm }

	def get_param(self, prm_name):
		return next( (prm for prm in self.params if prm['name'] == prm_name), None)

	def get_var(self, prm, var_name, def_ret=None):
		return next( (var for var in prm.get('vars') if var.get('name') == var_name), def_ret)

	def obj_to_cmd_dic(self, obj):
		return next( (cmd_dic for cmd_dic in self.all_cmd_dics if obj in cmd_dic), None)

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
		self.alias_sync(obj)
		obj = self.alias_grp_top_obj(obj)
		v = obj.GetValue()
		add_args = self.obj_to_add_args(obj, msg_box=v) # no open dialog at kill
		if add_args is False:
			set_val(obj, not v)
			return
		(cmd_dic, _, proc_bak) = self.obj_to_cmd_dic_cmd_proc(obj)
		self.launch_kill_proc(obj, cmd_dic, add_args=add_args)
		(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if proc != proc_bak:
			self.toggle_enable_obj(obj)
		if proc:
			self.update_proc_cpu(obj)

	def OnRosbagPlay(self, event):
		obj = event.GetEventObject()

		play = self.button_play_rosbag_play
		stop = self.button_stop_rosbag_play
		pause = self.button_pause_rosbag_play

		(_, _, prm) = self.obj_to_pdic_gdic_prm(play)
		var = self.get_var(prm, 'sim_time', {})

		if obj == play:
			var['v'] = True
			self.OnLaunchKill_obj(play)
			set_val(stop, False)
			set_val(pause, False)
		elif obj == stop:
			set_val(play, False)
			set_val(pause, False)
			var['v'] = False
			self.OnLaunchKill_obj(play)
		elif obj == pause:
			(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(play)
			if proc:
				proc.stdin.write(' ')
			
	def stdout_file_search(self, file, k):
		s = ''
		while True:
			c = file.read(1)
			if not c:
				return None
			if c != '\r' and c != '\n':
				s += c
				continue
			s = s.strip()
			if k in s:
				break
			s = ''
		i = s.find(k) + len(k)
		return s[i:]

	# thread
	def point_cloud_progress_bar(self, file, ev):
		obj = self.button_point_cloud
		(pdic, _, _) = self.obj_to_pdic_gdic_prm(obj)
		n = len(pdic.get('path_pcd', '').split(','))
		if n == 0:
			return
		i = 0
		self.pcd_loaded = False
		while not ev.wait(0):
			s = self.stdout_file_search(file, 'load ')
			if not s:
				break
			err_key = 'failed '
			if s[:len(err_key)] != err_key:
				i += 1
			else:
				i -= 1
				print s
			self.pcd_loaded = (i == n)
			wx.CallAfter(self.label_point_cloud_bar.set, 100 * i / n)
		wx.CallAfter(self.label_point_cloud_bar.clear)

	# thread
	def rosbag_play_progress_bar(self, file, ev):
		while not ev.wait(0):
			s = self.stdout_file_search(file, 'Duration:')
			if not s:
				break
			lst = s.split()
			pos = float(lst[0])
			# lst[1] is '/'
			total = float(lst[2])
			if total == 0:
				continue
			prg = int(100 * pos / total + 0.5)
			pos = str(int(pos))
			total = str(int(total))

			wx.CallAfter(self.label_rosbag_play_bar.set, prg)
			wx.CallAfter(self.label_rosbag_play_pos.SetLabel, pos)
			wx.CallAfter(self.label_rosbag_play_total.SetLabel, total)
		wx.CallAfter(self.label_rosbag_play_bar.clear)

	#def OnPauseRosbagPlay(self, event):
	#	pause_obj = event.GetEventObject()
	#	pause_obj = self.alias_grp_top_obj(pause_obj)
	#
	#	key = self.obj_key_get(pause_obj, ['button_pause_'])
	#	if not key:
	#		return
	#	obj = self.obj_get('button_launch_' + key)
	#	(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
	#	if proc:
	#		proc.stdin.write(' ')

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

	#def OnAliasSync(self, event):
	#	obj = event.GetEventObject()
	#	self.alias_sync(obj)

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
		return next( (grp for grp in self.alias_grps if obj in grp), [])

	def create_tree(self, parent, items, tree, item, cmd_dic):
		name = items.get('name', '')
		if tree is None:
			style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
			tree = CT.CustomTreeCtrl(parent, wx.ID_ANY, agwStyle=style)
			item = tree.AddRoot(name, data=tree)
		else:
			ct_type = 1 if 'cmd' in items else 0 # 1:checkbox type
			item = tree.AppendItem(item, name, ct_type=ct_type)
			if 'cmd' in items:
				cmd_dic[item] = (items['cmd'], None)

				pdic = self.load_dic_pdic_setup(name, items)
				pnl = wx.Panel(tree, wx.ID_ANY)
				add_objs = []
				self.new_link(item, name, pdic, self.sys_gdic, pnl, 'sys', 'sys', add_objs)
				gdic = self.gdic_get_1st(items)
				if 'param' in items:
					self.new_link(item, name, pdic, gdic, pnl, 'app', items.get('param'), add_objs)
				szr = sizer_wrap(add_objs, wx.HORIZONTAL, parent=pnl)
				szr.Fit(pnl)
				tree.SetItemWindow(item, pnl)
			self.try_setup_obj_stop(item, items);

		for sub in items.get('subs', []):
			self.create_tree(parent, sub, tree, item, cmd_dic)
		return tree

	def new_link(self, item, name, pdic, gdic, pnl, link_str, prm_name, add_objs):
		lkc = None
		if 'no_link' not in gdic.get('flags', []):
			lkc = wx.HyperlinkCtrl(pnl, wx.ID_ANY, link_str, "")
			fix_link_color(lkc)
			self.Bind(wx.EVT_HYPERLINK, self.OnHyperlinked, lkc)
			if len(add_objs) > 0:
				add_objs += [ wx.StaticText(pnl, wx.ID_ANY, ' ') ]
			add_objs += [ wx.StaticText(pnl, wx.ID_ANY, '['), lkc, wx.StaticText(pnl, wx.ID_ANY, ']') ]
		prm = self.get_param(prm_name)
		self.add_cfg_info(lkc if lkc else item, item, name, pdic, gdic, False, prm)

	def load_dic_pdic_setup(self, name, dic):
		name = dic.get('share_val', dic.get('name', name))
		pdic = self.load_dic.get(name, {})
		self.load_dic[ name ] = pdic
		return pdic

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
			obj = next( (obj for (obj, v) in cmd_dic.items() if proc in v), None)
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

			f = self.obj_to_gdic(obj, {}).get('stdout_func')
			f = eval(f) if type(f) is str else f
			f = f if f else self.log_th

			out = subprocess.PIPE if f else None
			err = subprocess.STDOUT if f else None
			if f == self.log_th:
				err = subprocess.PIPE

			proc = psutil.Popen(args, stdin=subprocess.PIPE, stdout=out, stderr=err)
			self.all_procs.append(proc)
			self.all_procs_nodes[ proc ] = self.nodes_dic.get(obj, [])

			if f == self.log_th:
				thinf = th_start(f, {'file':proc.stdout, 'que':self.log_que_stdout})
				self.all_th_infs.append(thinf)
				thinf = th_start(f, {'file':proc.stderr, 'que':self.log_que_stderr})
				self.all_th_infs.append(thinf)
			elif f:
				thinf = th_start(f, {'file':proc.stdout})
				self.all_th_infs.append(thinf)
		else:
			terminate_children(proc, sigint)
			terminate(proc, sigint)
			proc.wait()
			if proc in self.all_procs:
				self.all_procs.remove(proc)
				self.all_procs_nodes.pop(proc, None)
			proc = None

			stop_cmd = self.stop_dic.get(obj)
			if stop_cmd:
				subprocess.call( shlex.split(stop_cmd) )
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
		for cmd_dic in self.all_cmd_dics:
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
					add_args = self.obj_to_add_args(obj, msg_box=False)
					if add_args is False:
						continue
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
			s = subprocess.check_output(cmd).strip()
			return s.split('\n') if s != '' else []
		except subprocess.CalledProcessError:
			return []

	def set_bg_all_tabs(self, col=wx.NullColour):

		add_pnls = [
			self, 
			self.tree_ctrl_0,
			self.tree_ctrl_1,
			self.tree_ctrl_data ]

		for tab in self.all_tabs + add_pnls:
			tab.SetBackgroundColour(col)

	#def modal_dialog(self, lst, title=''):
	#	(lbs, cmds) = zip(*lst)
	#	dlg = MyDialog(self, lbs=lbs)
	#	dlg.SetTitle(title)
	#	r = dlg.ShowModal()
	#	ok = (0 <= r and r < len(cmds))
	#	return cmds[r] if ok else None

	def get_autoware_dir(self):
		dir = rtmgr_src_dir() + '../../../../../../'
		return os.path.abspath(dir)

	#def get_static_bitmap(self, parent, filename, scale):
	#	bm = self.get_bitmap(filename, scale)
	#	return wx.StaticBitmap(parent, wx.ID_ANY, bm)

	#def get_bitmap(self, filename, scale):
	#	dir = rtmgr_src_dir()
	#	bm = wx.Bitmap(dir + filename, wx.BITMAP_TYPE_ANY)
	#	(w, h) = bm.GetSize()
	#	img = wx.ImageFromBitmap(bm)
	#	img = img.Scale(w * scale, h * scale, wx.IMAGE_QUALITY_HIGH)
	#	bm = wx.BitmapFromImage(img)
	#	return bm

	def load_yaml(self, filename, def_ret=None):
		return load_yaml(filename, def_ret)

	def toggle_enable_obj(self, obj):
		objs = []
		pfs = [ 'button_launch_', 'button_kill_',
			'button_play_', 'button_stop_', 'button_pause_',
			'button_ref_', 'text_ctrl_' ]
		key = self.obj_key_get(obj, pfs)
		if key:
			objs += self.key_objs_get(pfs, key)
			
		gdic = self.obj_to_gdic(obj, {})
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
		return next( ( ( name[:len(pf)], name[len(pf):] ) for pf in pfs if name[:len(pf)] == pf ), None)

	def obj_key_get(self, obj, pfs):
		name = self.name_get(obj)
		if name is None:
			return None
		return next( (name[len(pf):] for pf in pfs if name[:len(pf)] == pf), None)

	def key_objs_get(self, pfs, key):
		return [ self.obj_get(pf + key) for pf in pfs if self.obj_get(pf + key) ]

	def name_get(self, obj):
		return next( (nm for nm in dir(self) if getattr(self, nm) is obj), None)

	def val_get(self, name):
		obj = self.obj_get(name)
		if obj is None:
			return None
		return obj.GetValue() if getattr(obj, 'GetValue', None) else None

	def obj_get(self, name):
		return getattr(self, name, None)

	def key_get(self, dic, val):
		return next( (k for (k,v) in dic.items() if v == val), None)

#class MyDialog(rtmgr.MyDialog):
#	def __init__(self, *args, **kwds):
#		lbs = kwds.pop('lbs')
#		rtmgr.MyDialog.__init__(self, *args, **kwds)
#
#		self.radio_box.Destroy()
#		self.radio_box = wx.RadioBox(self.panel_2, wx.ID_ANY, "", choices=lbs, majorDimension=0, style=wx.RA_SPECIFY_ROWS)
#
#		rtmgr.MyDialog.__set_properties(self)
#		rtmgr.MyDialog.__do_layout(self)
#
#	def __set_properties(self):
#		pass
#
#	def __do_layout(self):
#		pass
#
#	def OnOk(self, event):
#		ret = self.radio_box.GetSelection()
#		self.EndModal(ret)
#
#	def OnCancel(self, event):
#		self.EndModal(-1)

def gdic_dialog_type_chk(gdic, name):
	dlg_type = dic_list_get(gdic, 'dialog_type', 'config')
	other_key = 'open_dialog_only' if dlg_type == 'config' else 'config_dialog_only'
	other_lst = gdic.get(other_key, [])
	return False if name in other_lst else True

def gdic_dialog_name_get(gdic):
	dlg_type = dic_list_get(gdic, 'dialog_type', 'config')
	return gdic.get(dlg_type + '_dialog',  gdic.get('dialog', 'MyDialogParam') )

class ParamPanel(wx.Panel):
	def __init__(self, *args, **kwds):
		self.frame = kwds.pop('frame')
		self.pdic = kwds.pop('pdic')
		self.gdic = kwds.pop('gdic')
		self.prm = kwds.pop('prm')
		wx.Panel.__init__(self, *args, **kwds)

		obj = next( (v.get('obj') for (cfg_obj, v) in self.frame.config_dic.items() if v.get('param') is self.prm), None)
		(_, _, proc) = self.frame.obj_to_cmd_dic_cmd_proc(obj)

		hszr = None
		self.vps = []
		self.tmp_msg = None
		szr = wx.BoxSizer(wx.VERTICAL)

		topic_szrs = (None, None)

		vars = self.prm.get('vars')
		if self.gdic.get('show_order'):
			var_lst = lambda name, vars : [ var for var in vars if var.get('name') == name ]
			vars = reduce( lambda lst, name : lst + var_lst(name, vars), self.gdic.get('show_order'), [] )

		for var in vars:
			name = var.get('name')

			if not gdic_dialog_type_chk(self.gdic, name):
				continue

			if name not in self.gdic:
				self.gdic[ name ] = {}
			gdic_v = self.gdic.get(name)

			bak_stk_push(gdic_v, 'func')
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
			if vp.is_nl():
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

			user_category = gdic_v.get('user_category')
			if user_category is not None and hszr:
				user_szr = static_box_sizer(self, user_category, orient=wx.HORIZONTAL)
				(flgs, bdr) = gdic_v.get('user_category_add', [ [], 0 ])
				targ_szr.Add(user_szr, 0, wx_flag_get(flgs), bdr)
				targ_szr = hszr = user_szr

			targ_szr.Add(vp, prop, flag, border)

			if 'nl' in gdic_v.get('flags', []):
				hszr = None

			if do_category and self.in_msg(var):
				topic_szrs = (szr, hszr)
				(szr, hszr) = bak

			if 'hline' in gdic_v.get('flags', []) and hszr is None:
				szr.Add(wx.StaticLine(self, wx.ID_ANY), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 4)

			if not self.in_msg(var) and var.get('rosparam'):
				k = 'ext_toggle_enables'
				self.gdic[ k ] = self.gdic.get(k, []) + [ vp ]
				vp.Enable(proc is None)

			if 'disable' in gdic_v.get('flags', []):
				vp.Enable(False)
			if 'hide' in gdic_v.get('flags', []):
				vp.Hide()

		self.SetSizer(szr)
		if 'no_init_update' not in self.prm.get('flags', []):
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
				bak_stk_pop(gdic_v, 'func')

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
			style = wx.RA_SPECIFY_COLS if self.var.get('choices_style') == 'h' else wx.RA_SPECIFY_ROWS
			self.obj = wx.RadioBox(self, wx.ID_ANY, label, choices=choices, majorDimension=0, style=style)
			self.choices_sel_set(v)
			self.Bind(wx.EVT_RADIOBOX, self.OnUpdate, self.obj)
			return
		if self.kind == 'menu':
			choices = self.var.get('choices', [])
			self.obj = wx.Choice(self, wx.ID_ANY, choices=choices)
			self.choices_sel_set(v)
			self.Bind(wx.EVT_CHOICE, self.OnUpdate, self.obj)
			if label:
				lb = wx.StaticText(self, wx.ID_ANY, label)
				flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
				sizer_wrap((lb, self.obj), wx.HORIZONTAL, 0, flag, 4, self)
			return
		if self.kind == 'checkbox':
			self.obj = wx.CheckBox(self, wx.ID_ANY, label)
			self.obj.SetValue(v)
			self.Bind(wx.EVT_CHECKBOX, self.OnUpdate, self.obj)
			return
		if self.kind == 'checkboxes':
			item_n = self.var.get('item_n', 1)
			if type(item_n) is str:
				item_n = eval(item_n)
			self.obj = Checkboxes(self, item_n, label)
			self.obj.set(v)
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

		if self.kind == 'path':
			v = str(v)
			v = path_expand_cmd(v)
			v = os.path.expandvars(os.path.expanduser(v))

		self.tc = wx.TextCtrl(self, wx.ID_ANY, str(v), style=wx.TE_PROCESS_ENTER)
		self.Bind(wx.EVT_TEXT_ENTER, self.OnUpdate, self.tc)

		if self.kind in ('num', None):
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
		prop = 1 if self.kind == 'path' or self.kind == 'str' else 0
		szr.Add(self.tc, prop, flag, 4)

		if self.kind == 'path':
			self.ref = wx.Button(self, wx.ID_ANY, 'Ref')
			self.Bind(wx.EVT_BUTTON, self.OnRef, self.ref)
			self.ref.SetMinSize((40,29))
			szr.Add(self.ref, 0, flag, 4)

		if self.has_slider or self.kind == 'num':
			vszr = wx.BoxSizer(wx.VERTICAL)
			vszr.Add( self.create_bmbtn("inc.png", self.OnIncBtn) )
			vszr.Add( self.create_bmbtn("dec.png", self.OnDecBtn) )
			szr.Add(vszr, 0, wx.ALIGN_CENTER_VERTICAL)

		self.SetSizer(szr)

	def create_bmbtn(self, filename, hdr):
		dir = rtmgr_src_dir()
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
		if self.kind == 'checkboxes':
			return self.obj.get()
		if self.kind == 'hide':
			return self.var.get('v')
		if self.kind in [ 'path', 'str' ]:
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
			if self.has_slider:
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

	def is_nl(self):
		return self.has_slider or self.kind in [ 'path' ]

class MyDialogParam(rtmgr.MyDialogParam):
	def __init__(self, *args, **kwds):
		pdic = kwds.pop('pdic')
		self.pdic_bak = pdic.copy()
		gdic = kwds.pop('gdic')
		prm = kwds.pop('prm')
		rtmgr.MyDialogParam.__init__(self, *args, **kwds)

		self.Bind(wx.EVT_CLOSE, self.OnClose)

		ok_lb_key = 'open_dialog_ok_label'
		if dic_list_get(gdic, 'dialog_type', 'config') == 'open' and ok_lb_key in gdic:
			self.button_1.SetLabel( gdic.get(ok_lb_key) )

		parent = self.panel_v
		frame = self.GetParent()
		self.panel = ParamPanel(parent, frame=frame, pdic=pdic, gdic=gdic, prm=prm)
		szr = sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

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

	def OnClose(self, event):
		self.OnCancel(event)

class MyDialogDpm(rtmgr.MyDialogDpm):
	def __init__(self, *args, **kwds):
		pdic = kwds.pop('pdic')
		self.pdic_bak = pdic.copy()
		gdic = kwds.pop('gdic')
		prm = kwds.pop('prm')
		rtmgr.MyDialogDpm.__init__(self, *args, **kwds)

		self.Bind(wx.EVT_CLOSE, self.OnClose)

		parent = self.panel_v
		frame = self.GetParent()
		self.frame = frame
		self.panel = ParamPanel(parent, frame=frame, pdic=pdic, gdic=gdic, prm=prm)
		szr = sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

		self.SetTitle(prm.get('name', ''))
		(w,h) = self.GetSize()
		(w2,_) = szr.GetMinSize()
		w2 += 20
		if w2 > w:
			self.SetSize((w2,h))

		fix_link_color(self.hyperlink_car)
		fix_link_color(self.hyperlink_pedestrian)

	def OnOk(self, event):
		self.panel.update()
		self.panel.detach_func()
		self.EndModal(0)

	def OnLink(self, event):
		obj = event.GetEventObject()
		dic = { self.hyperlink_car : self.frame.button_launch_car_dpm,
			self.hyperlink_pedestrian : self.frame.button_launch_pedestrian_dpm }
		obj = dic.get(obj)
		if obj:
			self.frame.OnHyperlinked_obj(obj)

	def OnCancel(self, event):
		self.panel.pdic.update(self.pdic_bak) # restore
		self.panel.detach_func()
		self.panel.update()
		self.EndModal(-1)

	def OnClose(self, event):
		self.OnCancel(event)

class MyDialogCarPedestrian(rtmgr.MyDialogCarPedestrian):
	def __init__(self, *args, **kwds):
		pdic = kwds.pop('pdic')
		self.gdic = kwds.pop('gdic')
		prm = kwds.pop('prm')
		rtmgr.MyDialogCarPedestrian.__init__(self, *args, **kwds)

		self.Bind(wx.EVT_CLOSE, self.OnClose)

		frame = self.GetParent()
		self.frame = frame

		self.SetTitle(prm.get('name', ''))

		fix_link_color(self.hyperlink_car)
		fix_link_color(self.hyperlink_pedestrian)

	def OnLink(self, event):
		obj = event.GetEventObject()
		car_ped = { self.hyperlink_car : 'car', self.hyperlink_pedestrian : 'pedestrian' }.get(obj, 'car')
		obj_key = self.gdic.get('car_pedestrian_obj_key', {}).get(car_ped)
		obj = getattr(self.frame, 'button_launch_' + obj_key, None) if obj_key else None
		if obj:
			self.frame.OnHyperlinked_obj(obj)
		self.EndModal(0)

	def OnClose(self, event):
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

	def OnTrafficLightRecognition(self, event):
		pub = rospy.Publisher('/config/lane_stop', ConfigLaneStop, latch=True, queue_size=10)
		msg = ConfigLaneStop()
		if event.GetEventObject().GetValue():
			msg.manual_detection = False
		else:
			msg.manual_detection = True
		pub.publish(msg)

	def OnOk(self, event):
		self.EndModal(0)

	def OnCancel(self, event):
		self.EndModal(-1)

class MyDialogNdtMapping(rtmgr.MyDialogNdtMapping):
	def __init__(self, *args, **kwds):
		self.pdic = kwds.pop('pdic')
		self.pdic_bak = self.pdic.copy()
		self.gdic = kwds.pop('gdic')
		self.prm = kwds.pop('prm')
		rtmgr.MyDialogNdtMapping.__init__(self, *args, **kwds)

		parent = self.panel_v
		frame = self.GetParent()
		self.panel = ParamPanel(parent, frame=frame, pdic=self.pdic, gdic=self.gdic, prm=self.prm)
		sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

		self.update_filename()
		self.klass_msg = ConfigNdtMappingOutput
		self.pub = rospy.Publisher('/config/ndt_mapping_output', self.klass_msg, queue_size=10)

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
		self.panel.detach_func()
		self.EndModal(0)

class InfoBarLabel(wx.BoxSizer):
	def __init__(self, parent, btm_txt=None, lmt_bar_prg=90, bar_orient=wx.VERTICAL):
		wx.BoxSizer.__init__(self, orient=wx.VERTICAL)
		self.lb = wx.StaticText(parent, wx.ID_ANY, '')
		self.bar = BarLabel(parent, hv=bar_orient, show_lb=False)
		bt = wx.StaticText(parent, wx.ID_ANY, btm_txt) if btm_txt else None

		self.Add(self.lb, 0, wx.ALIGN_CENTER_HORIZONTAL, 0)
		if bar_orient == wx.VERTICAL:		
			sz = self.bar.GetSize()
			sz.SetWidth(20)
			self.bar.SetMinSize(sz)
			self.Add(self.bar, 1, wx.ALIGN_CENTER_HORIZONTAL, 0)
			if bt:
				self.Add(bt, 0, wx.ALIGN_CENTER_HORIZONTAL, 0)
		else:
			szr = wx.BoxSizer(wx.HORIZONTAL)
			if bt:
				szr.Add(bt, 0, 0, 0)
			szr.Add(self.bar, 1, 0, 0)
			self.Add(szr, 1, wx.EXPAND, 0)

		self.lmt_bar_prg = lmt_bar_prg

	def lb_set(self, txt, col):
		self.lb.SetForegroundColour(col)
		self.lb.SetLabel(txt);
		self.Layout()

	def bar_set(self, prg):
		(col1, col2) = (wx.Color(0,0,250), wx.Color(0,0,128))
		if prg >= self.lmt_bar_prg:
			(col1, col2) = (wx.Color(250,0,0), wx.Color(128,0,0)) 
		self.bar.set_col(col1, col2)
		self.bar.set(prg)

class Checkboxes(wx.Panel):
	def __init__(self, parent, item_n, lb):
		wx.Panel.__init__(self, parent, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize)
		self.boxes = [ wx.CheckBox(self, wx.ID_ANY, lb + str(i)) for i in range(item_n) ]
		vsz = wx.BoxSizer(wx.VERTICAL)
		for j in range((item_n + 7) / 8):
			hsz = wx.BoxSizer(wx.HORIZONTAL)
			for i in range(8):
				idx = j * 8 + i
				if idx < len(self.boxes):
					hsz.Add(self.boxes[idx], 0, wx.LEFT, 8)
			vsz.Add(hsz)
		self.SetSizer(vsz)
		vsz.Fit(self)

	def set(self, vs):
		vs = vs if vs else [ True for box in self.boxes ]
		for (box, v) in zip(self.boxes, vs):
			box.SetValue(v)

	def get(self):
		return [ box.GetValue() for box in self.boxes ]

class BarLabel(wx.Panel):
	def __init__(self, parent, txt='', pos=wx.DefaultPosition, size=wx.DefaultSize, style=0, hv=wx.HORIZONTAL, show_lb=True):
		wx.Panel.__init__(self, parent, wx.ID_ANY, pos, size)
		self.lb = wx.StaticText(self, wx.ID_ANY, '', style=style)
		self.txt = txt
		self.hv = hv
		self.dir = wx.SOUTH if hv == wx.HORIZONTAL else wx.EAST
		self.show_lb = show_lb
		self.prg = -1

		self.dflt_col1 = wx.Color(250,250,250)
		self.dflt_col2 = wx.Color(128,128,128)
		self.col1 = self.dflt_col1
		self.col2 = self.dflt_col2

		self.Bind(wx.EVT_PAINT, self.OnPaint)

	def set(self, prg):
		self.prg = prg
		if self.show_lb:
			self.lb.SetLabel(self.txt + str(prg) + '%' if prg >= 0 else '')
		self.Refresh()

	def set_col(self, col1, col2):
		self.col1 = col1 if col1 != wx.NullColour else self.dflt_col1
		self.col2 = col2 if col2 != wx.NullColour else self.dflt_col2

	def clear(self):
		self.set(-1)

	def OnPaint(self, event):
		dc = wx.PaintDC(self)
		(w,h) = self.GetSize()
		if self.IsEnabled():
			p = (w if self.hv == wx.HORIZONTAL else h) * self.prg / 100
			rect = wx.Rect(0, 0, p, h) if self.hv == wx.HORIZONTAL else wx.Rect(0, h-p, w, p)
			dc.GradientFillLinear(rect, self.col1, self.col2, self.dir)
			rect = wx.Rect(p, 0, w-p, h) if self.hv == wx.HORIZONTAL else wx.Rect(0, 0, w, h-p)
			dc.GradientFillLinear(rect, wx.Colour(200,200,200), wx.Colour(220,220,220), self.dir)
		else:
			rect = wx.Rect(0, 0, w, h)
			dc.GradientFillLinear(rect, wx.Colour(250,250,250), wx.Colour(250,250,250), self.dir)

class ColorLabel(wx.Panel):
	def __init__(self, parent, lst=[], pos=wx.DefaultPosition, size=wx.DefaultSize, style=0):
		wx.Panel.__init__(self, parent, wx.ID_ANY, pos, size)
		self.lst = lst
		self.Bind(wx.EVT_PAINT, self.OnPaint)

	def set(self, lst):
		self.lst = lst
		self.Refresh()

	def OnPaint(self, event):
		dc = wx.PaintDC(self)
		dc.Clear()

		#change_font_point_by_rate(dc, 0.75)

		(x,y) = (0,0)
		(_, h, _, _) = dc.GetFullTextExtent(' ')
		for v in self.lst:
			if type(v) is tuple and len(v) == 2:
				(x,y) = v
			elif type(v) is tuple and len(v) == 3:
				dc.SetTextForeground(v)
			elif v == '\n':
				(x,y) = (0,y+h)
			elif type(v) is str:
				dc.DrawText(v, x, y)
				(w, _, _, _) = dc.GetFullTextExtent(v)
				x += w

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
		self.toggles = [ self.button_start, self.button_stop ]

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

		split_arg = [ '--split' ] if self.checkbox_split.GetValue() else []
		size_arg = self.size_arg_get()
		if split_arg and not size_arg:
			wx.MessageBox('size is required, with split')
			return
		args += split_arg + size_arg

		(cmd, proc) = self.cmd_dic[ key_obj ]
		proc = self.parent.launch_kill(True, cmd, proc, add_args=args, obj=key_obj)
		self.cmd_dic[ key_obj ] = (cmd, proc)
		self.parent.toggle_enables(self.toggles)

	def OnStop(self, event):
		key_obj = self.button_start
		(cmd, proc) = self.cmd_dic[ key_obj ]
		proc = self.parent.launch_kill(False, cmd, proc, sigint=True, obj=key_obj)
		self.cmd_dic[ key_obj ] = (cmd, proc)
		self.parent.toggle_enables(self.toggles)
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
		self.update_filename()

	def update_filename(self):
		tc = self.text_ctrl
		path = tc.GetValue()
		(dn, fn) = os.path.split(path)
		now = datetime.datetime.now()
		fn = 'autoware-%04d%02d%02d%02d%02d%02d.rosbag' % (
			now.year, now.month, now.day, now.hour, now.minute, now.second)
		path = os.path.join(dn, fn)
		set_path(tc, path)

	def size_arg_get(self):
		tc = self.text_ctrl_size
		s = tc.GetValue()
		mb = 0
		try:
			mb = float(s)
		except ValueError:
			mb = 0
		if mb <= 0:
			tc.SetValue('')
		return [ '--size=' + str(int(mb * 1024 * 1024)) ] if mb > 0 else []

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
	dir = rtmgr_src_dir()
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
		terminate_children(child, sigint)
		terminate(child, sigint)

def terminate(proc, sigint=False):
	if sigint:
		proc.send_signal(signal.SIGINT)
	else:
		proc.terminate()

def th_start(target, kwargs={}):
	ev = threading.Event()
	kwargs['ev'] = ev
	th = threading.Thread(target=target, kwargs=kwargs)
	th.daemon = True
	th.start()
	return (th, ev)

def th_end((th, ev)):
	ev.set()
	th.join()

def que_clear(que):
	with que.mutex:
		que.queue.clear()

def append_tc_limit(tc, s, rm_chars=0):
	if rm_chars > 0:
		tc.Remove(0, rm_chars)
	tc.AppendText(s)

def cut_esc(s):
	while True:
		i = s.find(chr(27))
		if i < 0:
			break
		j = s.find('m', i)
		if j < 0:
			break
		s = s[:i] + s[j+1:]
	return s

def change_font_point_by_rate(obj, rate=1.0):
	font = obj.GetFont()
	pt = font.GetPointSize()
	pt = int(pt * rate)
	font.SetPointSize(pt)
	obj.SetFont(font)

def fix_link_color(obj):
	t = type(obj)
	if t is CT.GenericTreeItem or t is CT.CustomTreeCtrl:
		obj.SetHyperTextVisitedColour(obj.GetHyperTextNewColour())
	elif t is wx.HyperlinkCtrl:
		obj.SetVisitedColour(obj.GetNormalColour())

def scaled_bitmap(bm, scale):
	(w, h) = bm.GetSize()
	img = wx.ImageFromBitmap(bm)
	img = img.Scale(w * scale, h * scale, wx.IMAGE_QUALITY_HIGH)
	return wx.BitmapFromImage(img)

def sizer_wrap(add_objs, orient=wx.VERTICAL, prop=0, flag=0, border=0, parent=None):
	szr = wx.BoxSizer(orient)
	for obj in add_objs:
		szr.Add(obj, prop, flag, border)
	if parent:
		parent.SetSizer(szr)
	return szr

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
	if type(obj) is CT.GenericTreeItem:
		while obj.GetParent():
			obj = obj.GetParent()
		tree = obj.GetData()
		tree.Refresh()

# dic_list util (push, pop, get)
def dic_list_push(dic, key, v):
	if key not in dic:
		dic[key] = []
	dic.get(key).append(v)

def dic_list_pop(dic, key):
	dic.get(key, [None]).pop()

def dic_list_get(dic, key, def_ret=None):
	return dic.get(key, [def_ret])[-1]

def bak_stk_push(dic, key):
	if key in dic:
		k = key + '_bak_str'
		if k not in dic:
			dic[k] = []
		dic[k].append( dic.get(key) )

def bak_stk_pop(dic, key):
	k = key + '_bak_str'
	stk = dic.get(k, [])
	if len(stk) > 0:
		dic[key] = stk.pop()
	else:
		del dic[key]

def bak_stk_set(dic, key, v):
	bak_str_push(dic, key)
	dic[key] = v


def get_top(lst, def_ret=None):
	return lst[0] if len(lst) > 0 else def_ret

def adjust_num_str(s):
	if '.' in s:
		while s[-1] == '0':
			s = s[:-1]
		if s[-1] == '.':
			s = s[:-1]
	return s

def rtmgr_src_dir():
	return os.path.abspath(os.path.dirname(__file__)) + "/"

def path_expand_cmd(path):
	lst = path.split('/')
	s = lst[0]
	if s[:2] == '$(' and s[-1] == ')':
		cmd = s[2:-1].split(' ')
		lst[0] = subprocess.check_output(cmd).strip()
		path = '/'.join(lst)
	return path

def prn_dict(dic):
	for (k,v) in dic.items():
		print (k, ':', v)

def send_to_proc_manager(order):
	sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

	try:
		sock.connect(PROC_MANAGER_SOCK)
	except socket.error:
		print('Failed connect to {}'.format(PROC_MANAGER_SOCK))
		return -1

	sock.send(yaml.dump(order))
	ret = sock.recv(1024)
	sock.close()
	return int(ret) == 0

def set_process_nice(proc, value):
	order = {
		"name": "nice",
		"pid": proc.pid,
		"nice": value
	}
	return send_to_proc_manager(order)

def set_process_cpu_affinity(proc, cpus):
	order = {
		"name": "cpu_affinity",
		"pid": proc.pid,
		"cpus": cpus,
	}
	return send_to_proc_manager(order)

def shutdown_proc_manager():
	order = {
		"name": "shutdown",
	}
	return send_to_proc_manager(order)

def set_scheduling_policy(proc, policy, priority):
	order = {
		"name": "scheduling_policy",
		"pid": proc.pid,
		"policy": policy,
		"priority": priority,
	}
	return send_to_proc_manager(order)

if __name__ == "__main__":
	path = rtmgr_src_dir() + 'btnrc'
	if os.path.exists(path):
		gtk.rc_parse(path)

	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()

# EOF
