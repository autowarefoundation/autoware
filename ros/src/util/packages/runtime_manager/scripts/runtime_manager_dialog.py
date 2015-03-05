#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl as CT
import gettext
import os
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
from runtime_manager.msg import ConfigLaneFollower
from runtime_manager.msg import ConfigCarKf
from runtime_manager.msg import ConfigPedestrianKf
from runtime_manager.msg import ConfigLaneRule
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
		self.all_cmd_dics = []
		self.load_dic = self.load_yaml('param.yaml', def_ret={})
		self.config_dic = {}
		self.Bind(wx.EVT_CLOSE, self.OnClose)

		#
		# ros
		#
		rospy.init_node('runime_manager', anonymous=True)
		rospy.Subscriber('to_rtmgr', std_msgs.msg.String, self.RosCb)
		self.pub = rospy.Publisher('from_rtmgr', std_msgs.msg.String, queue_size=10)

		#
		# for Main tab (version)
		#
		tab = self.notebook_1_pane_1
		scale = 0.3
		self.bitmap_1 = self.get_static_bitmap(tab, "nagoya_university.png", scale)
		self.bitmap_2 = self.get_static_bitmap(tab, "axe.png", scale)

		self.main_cmd = {}
		self.all_cmd_dics.append(self.main_cmd)
		self.main_dic = self.load_yaml('main.yaml')

		self.params = []
		self.add_params(self.main_dic.get('params', []))

		self.load_yaml_button_run(self.main_dic.get('buttons', {}), self.main_cmd)

		self.main_cmd[ self.button_load_map ] = []

		self.route_cmd_waypoint = [ Waypoint(0,0), Waypoint(0,0) ]
		rospy.Subscriber('route_cmd', route_cmd, self.route_cmd_callback)

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

		self.create_checkboxes(dic, self.panel_sensing, None, self.drv_probe_cmd, self.sensing_cmd, self.OnSensingDriver)
		if 'buttons' in dic:
			self.load_yaml_button_run(dic['buttons'], self.sensing_cmd)

		# for button_calibration
		tc = self.text_ctrl_sensor_fusion
		path = os.path.expanduser("~") + '/.ros/autoware'
		tc.SetValue(path)
		tc.SetInsertionPointEnd()
		self.text_ctrl_calibration = tc
		self.button_ref_calibration = self.button_ref_sensor_fusion

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

		self.create_checkboxes(dic, self.panel_simulation, None, None, self.simulation_cmd, self.OnSimulation)
		if 'buttons' in dic:
			self.load_yaml_button_run(dic['buttons'], self.simulation_cmd)
		if 'checkboxs' in dic:
			self.load_yaml_button_run(dic['checkboxs'], self.simulation_cmd)

		self.vmap_names = self.load_yaml('vector_map_files.yaml')

		self.sel_multi_ks = [ 'point_cloud' ]
		self.sel_dir_ks = [ 'calibration', 'vector_map' ]

		self.set_param_panel(self.button_launch_pmap, self.panel_pmap_prm)
		self.set_param_panel(self.button_launch_vmap, self.panel_vmap_prm)
		self.set_param_panel(self.button_launch_trajectory, self.panel_trajectory_prm)

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
			self.load_yaml_button_run(dic['buttons'], self.data_cmd)

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
			[ self.button_launch_tf, self.button_main_tf, ],
			[ self.button_kill_tf, self.button_main_tf, ],
			[ self.button_ref_tf, self.button_ref_main_tf, ],
			[ self.text_ctrl_tf, self.text_ctrl_main_tf, ],
			[ self.button_launch_rosbag_play, self.button_launch_main_rosbag_play, ],
			[ self.button_kill_rosbag_play, self.button_kill_main_rosbag_play, ],
			[ self.button_pause_rosbag_play, self.button_pause_main_rosbag_play, ],
			[ self.text_ctrl_file_rosbag_play, self.text_ctrl_main_rosbag_play, ],
			[ self.button_ref_file_rosbag_play, self.button_ref_main_rosbag_play, ],
			[ self.text_ctrl_rate_rosbag_play, self.text_ctrl_rate_main_rosbag_play, ],
			[ self.checkbox_clock_rosbag_play, self.checkbox_clock_main_rosbag_play, ],
			[ self.checkbox_sim_time, self.checkbox_main_sim_time, ],
		]

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
			f = open(dir + 'param.yaml', 'w')
			s = yaml.dump(save_dic, default_flow_style=False)
			print 'save\n', s # for debug
			f.write(s)
			f.close()

		shutdown_sh = self.get_autoware_dir() + '/shutdown.sh'
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

	def load_yaml_button_run(self, d, run_dic):
		for (k,d2) in d.items():
			obj = get_top( self.key_objs_get([ 'button_', 'button_launch_', 'checkbox_' ], k) )
			if not obj:
				print('xxx_' + k + ' not found correspoinding widget.')
				continue
			if not d2 or type(d2) is not dict:
				continue
			if 'run' in d2:
				run_dic[obj] = (d2['run'], None)
			if 'param' in d2:
				pdic = self.load_dic.get(k)
				if pdic is None:
					pdic = {}
					self.load_dic[k] = pdic
				prm = self.get_param(d2.get('param'))
				gdic = self.gdic_get_1st(d2)
				self.add_cfg_info(obj, obj, k, pdic, gdic, False, prm)

	#
	# Main Tab
	#
	def OnMainButton(self, event):
		obj = event.GetEventObject()
		(cmd, _) = self.main_cmd.get(obj, (None, None))
		args = shlex.split(cmd)
		print(args)
		proc = subprocess.Popen(args, stdin=subprocess.PIPE)
		self.all_procs.append(proc)

	def OnStart(self, event):
		#cmd = 'rostopic pub -1 error_info ui_socket/error_info \'{header: {seq: 0, stamp: 0, frame_id: ""}, error: 1}\''
		cmd = 'roslaunch ' + self.get_autoware_dir() + '/autoware_start.launch'
		print(cmd)
		os.system(cmd)

	def OnDrive(self, event):
		pub = rospy.Publisher('mode_cmd', mode_cmd, queue_size=10)
		pub.publish(mode_cmd(mode=1))

	def OnPause(self, event):
		pub = rospy.Publisher('mode_cmd', mode_cmd, queue_size=10)
		pub.publish(mode_cmd(mode=0))

	def OnStop(self, event):
		#cmd = 'rostopic pub -1 error_info ui_socket/error_info \'{header: {seq: 0, stamp: 0, frame_id: ""}, error: 0}\''
		self.kill_all()

	def OnNetConn(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.main_cmd)

	def OnReadNavi(self, event):
		self.text_ctrl_route_from_lat.SetValue(str(self.route_cmd_waypoint[0].lat))
		self.text_ctrl_route_from_lon.SetValue(str(self.route_cmd_waypoint[0].lon))
		self.text_ctrl_route_to_lat.SetValue(str(self.route_cmd_waypoint[1].lat))
		self.text_ctrl_route_to_lon.SetValue(str(self.route_cmd_waypoint[1].lon))

	def OnTextRoute(self, event):
		pass

	def OnLoadMap(self, event):
		obj = event.GetEventObject()
		v = obj.GetValue()
		procs = self.main_cmd.get(obj, [])
		if (v and procs != []) or (not v and procs == []):
			obj.SetValue(not v)
			return
		if v:
			path_area_list = self.text_ctrl_area_list.GetValue()
			path_pcd = self.text_ctrl_point_cloud.GetValue()
			path_pcd = path_pcd.split(',')
			auto = self.checkbox_auto_update.GetValue()

			cmd = None
			if auto and path_area_list != '':
				cmd = 'rosrun map_file points_map_loader'
			if not auto and path_pcd != []:
				cmd = 'rosrun sample_data sample_points_map'
				path_area_list = ''
			if cmd:
				add_args = [ path_area_list ] if path_area_list != '' else []
				add_args += path_pcd
				print cmd, add_args
				proc = self.launch_kill(v, cmd, None, add_args)
				procs += [ proc ]

			path_vec = self.text_ctrl_vector_map.GetValue()
			if path_vec != '':
				path_vec = [ path_vec + '/' + nm for nm in self.vmap_names ]
				cmd = 'rosrun sample_data sample_vector_map'
				add_args = path_vec + [ 'swap_x_y_on' ]
				print cmd, add_args
				proc = self.launch_kill(v, cmd, None, add_args)
				procs += [ proc ]
		else:
			for proc in procs:
				self.launch_kill(v, 'dmy', proc)
			procs = []
		self.main_cmd[ obj ] = procs

	def OnMainTf(self, event):
		obj = event.GetEventObject()
		v = obj.GetValue()
		if v:
			self.OnLaunch_obj(self.button_launch_tf)
		else:
			self.OnKill_kill_obj(self.button_kill_tf)

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

	def route_cmd_callback(self, data):
		self.route_cmd_waypoint = data.point

	#
	# Computing Tab
	#
	def OnTreeChecked(self, event):
		self.OnChecked_obj(event.GetItem())

	def OnChecked_obj(self, obj):
		cmd_dic = self.obj_to_cmd_dic(obj)
		add_args = self.obj_to_add_args(obj)
		self.launch_kill_proc(obj, cmd_dic, add_args=add_args)

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
				name = cmd_param.get('var_name', name)
				if cmd_param.get('only_enable') and not v:
					continue
				unpack = cmd_param.get('unpack')
				if unpack is not None:
					v = ' '.join( v.split(unpack) )
				add = ''
				dash = cmd_param.get('dash')
				if dash is not None:
					add += dash + name
				delim = cmd_param.get('delim')
				if delim is not None:
					add += delim + str(v)
				if add != '':
					s += add + ' '
		return s.strip(' ').split(' ') if s != '' else None

	def obj_to_pdic_gdic_prm(self, obj):
		info = self.config_dic.get(obj)
		if info is None:
			info = get_top([ v for v in self.config_dic.values() if v.get('obj') is obj ])
			if info is None:
				return (None, None)
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

	def publish_param_topic(self, pdic, prm):
		pub = prm['pub']
		klass_msg = globals()[ prm['msg'] ]
		msg = klass_msg()

		for (name, v) in pdic.items():
			(obj, attr) = msg_path_to_obj_attr(msg, name)
			if attr in obj.__slots__:
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
			if rosparam in rosparams:
				cmd = [ 'rosparam', 'get', rosparam ]
				ov = subprocess.check_output(cmd).strip()
				if ov == v:
					continue
			cmd = [ 'rosparam', 'set', rosparam, v ]
			print(cmd)
			subprocess.call(cmd)

	def OnRefresh(self, event):
		subprocess.call([ 'sh', '-c', 'echo y | rosnode cleanup' ])
		run_nodes = subprocess.check_output([ 'rosnode', 'list' ]).strip().split('\n')
		run_nodes_set = set(run_nodes)
		for (obj, nodes) in self.nodes_dic.items():
			v = nodes and run_nodes_set.issuperset(nodes)
			if obj.GetValue() != v:
				set_check(obj, v)
				obj.Enable(not v)
		self.Refresh()

	#
	# Viewer Tab
	#
	def create_viewer_btns(self, parent, sizer, lst):
		for dic in lst:
			lb = dic['label']
			flag = wx.ALL
			if 'subs' in dic:
				sb = wx.StaticBox(parent, wx.ID_ANY, lb)
				sb.Lower()
				obj = wx.StaticBoxSizer(sb, wx.VERTICAL)
				self.create_viewer_btns(parent, obj, dic['subs'])
			else:
				obj = wx.ToggleButton(parent, wx.ID_ANY, lb)
				self.Bind(wx.EVT_TOGGLEBUTTON, self.OnViewer, obj)
				self.viewer_cmd[obj] = (dic['cmd'], None)
				flag |= wx.EXPAND
			sizer.Add(obj, 0, flag, 4)

	def OnViewer(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.viewer_cmd)

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.OnChecked_obj(event.GetEventObject())

	def OnCalib(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.sensing_cmd)

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
				sb = wx.StaticBox(panel, wx.ID_ANY, dic['name'])
				sb.Lower()
				obj = wx.StaticBoxSizer(sb, wx.VERTICAL)
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
		pdic = self.load_dic.get(name, None)
		if pdic is None:
			pdic = {}
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
			cmd = cmd.get(obj.GetValue(), None)
		if cmd:
			print(cmd)
			os.system(cmd)

	def OnPointMapUpdate(self, event):
		key = 'pmap'
		obj = self.obj_get('button_launch_' + key)
		kill_obj = self.obj_get('button_kill_' + key)
		if obj.IsEnabled() or not kill_obj.IsEnabled():
			return
		self.OnKill_kill_obj(kill_obj)
		self.OnLaunch_obj(obj)

	#
	# Data Tab
	#
	def check_download_objects_stat(self, btn, v, tcs, add_args):
		ngs = []
		if v:
			for s in add_args:
				try:
					float(s)
				except ValueError:
					ngs.append(tcs[ add_args.index(s) ])
		if len(ngs) == 0:
			for tc in tcs:
				tc.Enable(not v)
			return True

		for tc in ngs:
			tc.SetValue('')
		btn.SetValue(False)
		btn.Disable()
		return False

	#
	# Common Utils
	#
	def set_param_panel(self, obj, parent):
		(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
		panel = ParamPanel(parent, frame=self, pdic=pdic, gdic=gdic, prm=prm)
		szr = wx.BoxSizer(wx.VERTICAL)
		szr.Add(panel, 0, wx.EXPAND)
		parent.SetSizer(szr)

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

		key = self.obj_key_get(obj, ['button_launch_'])
		if not key:
			return
		tc = self.obj_get('text_ctrl_' + key) 
		path = tc.GetValue() if tc else None

		if key == 'tf' and not path:
			# TF default setting
			path = 'runtime_manager,tf.launch' # !

		if tc and not path:
			return

		(cmd_dic, cmd, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
		if cmd_dic is None or cmd is None:
			return

		if add_args and key == 'vmap' and self.vmap_names:
			add_args = [ add_args[0] + '/' + nm for nm in self.vmap_names ] + add_args[1:]
			
		if path:
			add_args = ( add_args if add_args else [] ) + path.split(',')

		if key == 'download':
			pf = 'text_ctrl_moving_objects_route_'
			lst = [ 'to_lat', 'to_lon', 'from_lat', 'from_lon' ]
			tcs = [ self.obj_get(pf + nm) for nm in lst ]
			add_args = [ tc.GetValue() for tc in tcs ]
			if not self.check_download_objects_stat(obj, True, tcs, add_args):
				return

		proc = self.launch_kill(True, cmd, proc, add_args)
		cmd_dic[obj] = (cmd, proc)

		self.enable_key_objs([ 'button_kill_', 'button_pause_' ], key)
		self.enable_key_objs([ 'button_launch_', 'text_ctrl_', 'button_ref_', 'text_ctrl_rate_', 
				       'checkbox_clock_' ], key, en=False)

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

		proc = self.launch_kill(False, cmd, proc, sigint=sigint)
		cmd_dic[obj] = (cmd, proc)

		if key == 'download':
			pf = 'text_ctrl_moving_objects_route_'
			lst = [ 'to_lat', 'to_lon', 'from_lat', 'from_lon' ]
			tcs = [ self.obj_get(pf + nm) for nm in lst ]
			self.check_download_objects_stat(obj, False, tcs, [])

		self.enable_key_objs([ 'button_launch_', 'text_ctrl_', 'button_ref_', 'text_ctrl_rate_', 
				       'checkbox_clock_' ], key)
		self.enable_key_objs([ 'button_kill_', 'button_pause_' ], key, en=False)

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
		b = event.GetEventObject()
		nm = self.name_get(b) # button_ref_xxx
		k = nm[ len('button_ref_'): ] # xxx
		tc = self.obj_get('text_ctrl_' + k)
		if tc is None:
			return
		path = tc.GetValue()
		multi = k in self.sel_multi_ks
		dir = k in self.sel_dir_ks
		save = k in [ 'rosbag_record' ]
		path = self.file_dialog(path, dir, multi, save)
		if path:
			tc.SetValue(path)
			tc.SetInsertionPointEnd()
			self.alias_sync(tc)

	def OnAliasSync(self, event):
		obj = event.GetEventObject()
		self.alias_sync(obj)

	def alias_sync(self, obj, v=None):
		en = obj.IsEnabled()
		grp = self.alias_grp_get(obj)
		if getattr(obj, 'GetValue', None):
			v = obj.GetValue()
		for o in grp:
			if o is obj:
				continue
			
			if o.IsEnabled() != en and not self.is_toggle_button(o):
				o.Enable(en)
			if v is not None and getattr(o, 'SetValue', None):
				o.SetValue(v)
				if getattr(o, 'SetInsertionPointEnd', None):
					o.SetInsertionPointEnd()
			o.GetParent().Refresh()

	def alias_grp_top_obj(self, obj):
		return get_top(self.alias_grp_get(obj), obj)

	def alias_grp_get(self, obj):
		return get_top([ grp for grp in self.alias_grps if obj in grp ], [])

	def file_dialog(self, defaultPath='', dir=False, multi=False, save=False):
		if dir:
			dlg = wx.DirDialog(self, defaultPath=defaultPath);
			multi = False
		else:
			(dn, fn) = os.path.split(defaultPath)
			style = wx.FD_SAVE if save else wx.FD_MULTIPLE if multi else wx.FD_DEFAULT_STYLE 
			dlg = wx.FileDialog(self, defaultDir=dn, defaultFile=fn, style=style)
		path = None
		if dlg.ShowModal() == wx.ID_OK:
			path = ','.join(dlg.GetPaths()) if multi else dlg.GetPath()
		dlg.Destroy()
		return path

	def create_tree(self, parent, items, tree, item, cmd_dic):
		name = items.get('name', '')
		if tree is None:
			style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
			tree = CT.CustomTreeCtrl(parent, wx.ID_ANY, style=style)
			item = tree.AddRoot(name)
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
		pdic = self.load_dic.get(name, None)
		self.add_cfg_info(item, item, name, pdic, gdic, False, prm)
		item.SetHyperText()

	def launch_kill_proc_file(self, obj, cmd_dic, names=None):
		key = self.obj_key_get(obj, [ 'button_', 'checkbox_' ])
		if key is None:
			return
		v = obj.GetValue()
		tc = self.obj_get('text_ctrl_' + key)

		path = tc.GetValue()
		if v and not path:
			obj.SetValue(False)
			return
		add_args = [ path + '/' + nm for nm in names ] if names else path.split(',')
		self.launch_kill_proc(obj, cmd_dic, add_args)
		en = not obj.GetValue()
		tc.Enable(en)
		ref = self.obj_get('button_ref_' + key)
		ref.Enable(en)

	def launch_kill_proc(self, obj, cmd_dic, add_args=None):
		if obj not in cmd_dic:
			set_check(obj, False)
			print('not implemented.')
			return
		v = obj.GetValue()
		(cmd, proc) = cmd_dic[obj]
		if not cmd:
			set_check(obj, False)
		cmd_bak = cmd
		if v and type(cmd) is list:
			cmd = self.modal_dialog(obj, cmd)
			if not cmd:
				return # cancel

		proc = self.launch_kill(v, cmd, proc, add_args)

		cfg_obj = self.get_cfg_obj(obj)
		if cfg_obj and self.config_dic[ cfg_obj ]['run_disable']:
			cfg_obj.Enable(not v)

		cmd_dic[obj] = (cmd_bak, proc)

	def kill_all(self):
		all = self.all_procs[:] # copy
		for proc in all:
			(cmd_dic, obj) = self.proc_to_cmd_dic_obj(proc)
			if obj:
				(cmd, _) = cmd_dic[ obj ]
				if type(cmd) is dict:
					cmd = self.selobj_cmd_get(cmd)
				print('kill ' + str(cmd))

				key = self.obj_key_get(obj, [ 'button_launch_' ])
				if key:
					self.OnKill_kill_obj(self.obj_get('button_kill_' + key))
					return
				self.cmd_dic_obj_off_for_kill(cmd_dic, obj)
			self.launch_kill(False, 'dmy', proc)

	def cmd_dic_obj_off_for_kill(self, cmd_dic, obj):
		set_check(obj, False)
		v = cmd_dic[ obj ]
		if type(v) is list:
			v.remove(obj)
		else:
			(cmd, _) = cmd_dic[ obj ]
			cmd_dic[ obj ] = (cmd, None)

	def proc_to_cmd_dic_obj(self, proc):
		for cmd_dic in self.all_cmd_dics:
			obj = get_top( [ obj for (obj, v) in cmd_dic.items() if proc in v ] )
			if obj:
				return (cmd_dic, obj)
		return (None, None)

	def launch_kill(self, v, cmd, proc, add_args=None, sigint=False):
		msg = None
		msg = 'already launched.' if v and proc else msg
		msg = 'already terminated.' if not v and proc is None else msg
		msg = 'cmd not implemented.' if not cmd else msg
		if msg is not None:
			print(msg)
			return proc

		if v and type(cmd) is dict:
			cmd = self.selobj_cmd_get(cmd)
		if v:
			t = cmd
			# for replace
			if t.find('replace') >= 0:
				t2 = eval(t)
				if t2 != t:
					t = t2
					add_args = None

			args = shlex.split(t)
			if add_args:
				s = '__args__'
				pos = args.index(s) if s in args else -1
				args = args[0:pos] + add_args + args[pos+1:] if pos >= 0 else args + add_args
			print(args) # for debug
			proc = subprocess.Popen(args, stdin=subprocess.PIPE)
			self.all_procs.append(proc)
		else:
			terminate_children(proc, sigint)
			terminate(proc, sigint)
			proc.wait()
			if proc in self.all_procs:
				self.all_procs.remove(proc)
			proc = None
		return proc

	def selobj_cmd_get(self, cmd):
		if type(cmd) is dict:
			selobj = self.obj_get(cmd['selobj'])
			selkey = selobj.GetValue() if selobj else None
			cmd = cmd.get(selkey, 'not found selkey=' + str(selkey))
		return cmd
		
	def nodes_dic_get(self):
		nodes_dic = {}
		#for cmd_dic in self.all_cmd_dics:
		for cmd_dic in [ self.computing_cmd, self.data_cmd ]:
			for (obj, (cmd, _)) in cmd_dic.items():
				nodes_dic[ obj ] = self.cmd_to_nodes(cmd)
		return nodes_dic

	def cmd_to_nodes(self, cmd):
		if not cmd:
			return None
		if type(cmd) is list:
			return reduce(lambda a,b : a+b, [ self.cmd_to_nodes(c) for c in cmd ])
		if type(cmd) is dict:
			return self.cmd_to_ndoes(self.selobj_cmd_get(cmd))

		cmd = shlex.split(cmd)
		if cmd[0] == 'roslaunch':
			cmd.insert(1, '--node')
			try:
				return subprocess.check_output(cmd).strip().split('\n')
			except subprocess.CalledProcessError:
				return None
			
		elif cmd[0] == 'rosrun':
			return [ '/' + cmd[2] ]
		return None

	def modal_dialog(self, obj, lst):
		(lbs, cmds) = zip(*lst)
		dlg = MyDialog(self, lbs=lbs)
		dlg.SetTitle(obj.GetLabel())
		r = dlg.ShowModal()
		ok = (0 <= r and r < len(cmds))
		if not ok:
			obj.SetValue(False)
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
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		path = dir + filename
		if not os.path.isfile(path):
			return def_ret
		f = open(dir + filename, 'r')
		d = yaml.load(f)
		f.close()
		return d

	def enable_key_objs(self, pfs, key, en=True):
		for obj in self.key_objs_get(pfs, key):
			obj.Enable(en)
			self.alias_sync(obj)

	def is_toggle_button(self, obj):
		return self.name_get(obj).split('_')[0] == 'button' and getattr(obj, 'GetValue', None)

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

		hszr = None
		self.vps = []
		szr = wx.BoxSizer(wx.VERTICAL)
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

			gdic_v['func'] = vp.get_v
			prop = gdic_v.get('prop', 0)
			border = gdic_v.get('border', 0)
			flag = wx_flag_get(gdic_v.get('flags', []))

			if vp.has_slider or vp.kind =='path':
				hszr = None if hszr else hszr
				flag |= wx.EXPAND
				szr.Add(vp, prop, flag, border)
			else:
				if hszr is None:
					hszr = wx.BoxSizer(wx.HORIZONTAL)
					szr.Add(hszr, 0, wx.EXPAND)
				flag |= wx.ALIGN_CENTER_VERTICAL
				hszr.Add(vp, prop, flag, border)

			if 'nl' in var.get('flags', []):
				hszr = None

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

class VarPanel(wx.Panel):
	def __init__(self, *args, **kwds):
		self.var = kwds.pop('var')
		v = kwds.pop('v')
		self.update = kwds.pop('update')
		wx.Panel.__init__(self, *args, **kwds)

		self.min = self.var.get('min', None)
		self.max = self.var.get('max', None)
		self.has_slider = self.min is not None and self.max is not None

		label = self.var.get('label', '')
		self.kind = self.var.get('kind', None)
		if self.kind == 'radio_box':
			choices = self.var.get('choices', [])
			self.obj = wx.RadioBox(self, wx.ID_ANY, label, choices=choices, majorDimension=0, style=wx.RA_SPECIFY_ROWS)
			self.obj.SetSelection(v)
			self.Bind(wx.EVT_RADIOBOX, self.OnUpdate, self.obj)
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
		flag = wx.TOP | wx.BOTTOM | wx.LEFT | wx.ALIGN_CENTER_VERTICAL
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
				szr.Add(self.slider, 1, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
			else:
				self.is_float = type(self.var['v']) is not int
				self.tc.SetMinSize((40,27))

		flag = wx.TOP | wx.BOTTOM | wx.ALIGN_CENTER_VERTICAL
		prop = 1 if self.kind == 'path' else 0
		szr.Add(self.tc, prop, flag, 4)

		if self.kind == 'path':
			self.ref = wx.Button(self, wx.ID_ANY, 'Ref')
			self.Bind(wx.EVT_BUTTON, self.OnRef, self.ref)
			self.ref.SetMinSize((40,29))
			szr.Add(self.ref, 0, flag, 4)

		self.SetSizer(szr)

	def get_v(self):
		if self.kind == 'radio_box':
			return self.obj.GetSelection()
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
		self.tc.SetValue(str(v))
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
			s = str(Decimal(v).quantize(Decimal('.01')))
		self.tc.SetValue(s)
		self.update()

	def OnUpdate(self, event):
		if self.has_slider:
			self.slider.SetValue(self.get_int_v())
		self.update()

	def OnRef(self, event):
		path = self.tc.GetValue()
		(dn, fn) = os.path.split(path)
		path_type = self.var.get('path_type', None)
		if path_type == 'dir':
			dlg = wx.DirDialog(self, defaultPath=path)
		else:
			st_dic = { 'save' : wx.FD_SAVE, 'multi' : wx.FD_MULTIPLE }
			dlg = wx.FileDialog(self, defaultDir=dn, defaultFile=fn, 
					    style=st_dic.get(path_type, wx.FD_DEFAULT_STYLE))
		if dlg.ShowModal() == wx.ID_OK:
			path = ','.join(dlg.GetPaths()) if path_type == 'multi' else dlg.GetPath()
			self.tc.SetValue(path)
			self.tc.SetInsertionPointEnd()
			self.upate()
		dlg.Destroy()

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
		path = tc.GetValue()
		(dn, fn) = os.path.split(path)
		dlg = wx.FileDialog(self, defaultDir=dn, defaultFile=fn, style=wx.FD_SAVE)
		if dlg.ShowModal() == wx.ID_OK:
			tc.SetValue(dlg.GetPath())
			tc.SetInsertionPointEnd()

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
		proc = self.parent.launch_kill(True, cmd, proc, add_args=args)
		self.cmd_dic[ key_obj ] = (cmd, proc)

	def OnStop(self, event):
		key_obj = self.button_start
		(cmd, proc) = self.cmd_dic[ key_obj ]
		proc = self.parent.launch_kill(False, cmd, proc, sigint=True)
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
		tc.SetValue(path)
		tc.SetInsertionPointEnd()

def terminate_children(proc, sigint=False):
	for child in psutil.Process(proc.pid).get_children():
		terminate(child, sigint)

def terminate(proc, sigint=False):
	if sigint:
		proc.send_signal(signal.SIGINT)
	else:
		proc.terminate()

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
	t = cvt_dic.get(type_str, None)
	return t(str) if t else def_ret

def set_check(obj, v):
	func = getattr(obj, 'SetValue', getattr(obj, 'Check', None))
	if func:
		func(v)

def get_top(lst, def_ret=None):
	return lst[0] if len(lst) > 0 else def_ret

def prn_dict(dic):
	for (k,v) in dic.items():
		print (k, ':', v)

if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()

# EOF
