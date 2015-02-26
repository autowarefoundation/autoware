#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl as CT
import wx.lib.buttons
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
from ui_socket.msg import mode_cmd
from ui_socket.msg import gear_cmd
from runtime_manager.msg import accel_cmd
from runtime_manager.msg import steer_cmd
from runtime_manager.msg import brake_cmd

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

		scale = 0.5
		bm_on = self.get_bitmap('btnon.png', scale)
		bm_off = self.get_bitmap('btnoff.png', scale)

		for nm in [ 'tablet', 'mobile', 'vehicle', 'database' ]:
			setattr(self, 'bitmap_' + nm, self.get_static_bitmap(tab, nm+'.png', 0.3))

			getattr(self, 'button_' + nm).Destroy()
			btn = wx.lib.buttons.GenBitmapToggleButton(tab, wx.ID_ANY, bm_off)
			btn.SetBitmapSelected(bm_on)
			self.Bind(wx.EVT_BUTTON, self.OnNetConn, btn)
			setattr(self, 'button_' + nm, btn)

		self.main_cmd = {}
		self.all_cmd_dics.append(self.main_cmd)
		self.main_dic = self.load_yaml('main.yaml')
		self.load_yaml_button_run(self.main_dic.get('buttons', {}), self.main_cmd)

		self.main_cmd[ self.button_load_map ] = []

		#
		# for Computing tab
		#
		parent = self.tree_ctrl_0.GetParent()
		for i in range(3):
			self.obj_get('tree_ctrl_' + str(i)).Destroy()
		items = self.load_yaml('computing_launch_cmd.yaml')

		self.params = items.get('params', [])
		for prm in self.params:
			if 'topic' in prm and 'msg' in prm:
				klass_msg = globals()[ prm['msg'] ]
				prm['pub'] = rospy.Publisher(prm['topic'], klass_msg, queue_size=10)

		self.computing_cmd = {}
		self.all_cmd_dics.append(self.computing_cmd)
		for i in range(3):
			tree_ctrl = self.create_tree(parent, items['subs'][i], None, None, self.computing_cmd)
			tree_ctrl.ExpandAll()
			tree_ctrl.SetHyperTextVisitedColour(tree_ctrl.GetHyperTextNewColour()) # no change
			setattr(self, 'tree_ctrl_' + str(i), tree_ctrl)

		self.setup_config_param_pdic()

		self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)
		self.Bind(CT.EVT_TREE_ITEM_HYPERLINK, self.OnTreeHyperlinked)

		rtmgr.MyFrame.__do_layout(self)

		#
		# for Main Tab
		#
		self.sock_a = None
		self.sock_b = None
		self.sock_c = None
		self.sock_d = None

		#
		# for Sensing Tab
		#
		self.drv_probe_cmd = {}
		self.sensing_cmd = {}
		self.all_cmd_dics.append(self.sensing_cmd)
		dic = self.load_yaml('sensing.yaml')
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
		self.create_checkboxes(dic, self.panel_simulation, None, None, self.simulation_cmd, self.OnSimulation)
		if 'buttons' in dic:
			self.load_yaml_button_run(dic['buttons'], self.simulation_cmd)
		if 'checkboxs' in dic:
			self.load_yaml_button_run(dic['checkboxs'], self.simulation_cmd)

		self.vmap_names = self.load_yaml('vector_map_files.yaml')

		self.sel_multi_ks = [ 'pmap', 'point_cloud' ]
		self.sel_dir_ks = [ 'vmap', 'calibration', 'vector_map' ]

		#
		# for Data Tab
		#
		self.data_cmd = {}
		self.all_cmd_dics.append(self.data_cmd)
		dic = self.load_yaml('data.yaml')
		if 'buttons' in dic:
			self.load_yaml_button_run(dic['buttons'], self.data_cmd)

		vehicle_cbxs = dic.get('vehicle', [])
		szr = None
		for d in vehicle_cbxs:
			name = d.get('name', None)
			if not name:
				continue
			if not szr:
				szr = wx.BoxSizer(wx.HORIZONTAL)
				self.sizer_on_the_vehicle.Add(szr, 0, wx.ALL, 4)
			cbx = wx.CheckBox(self.notebook_1_pane_4, wx.ID_ANY, d.get('label', ''))
			szr.Add(cbx, 0, wx.ALL, 4)
			setattr(self, name, cbx)
			if vehicle_cbxs.index(d) % 3 == 2:
				szr = None

		#
		# for Viewer Tab
		#
		self.viewer_cmd = {}
		self.all_cmd_dics.append(self.viewer_cmd)
		parent = self.panel_viewer
		sizer = self.sizer_viewer
		lst = self.load_yaml('viewer.yaml', {}).get('viewers', [])
		self.create_viewer_btns(parent, sizer, lst)

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

	def OnTextIp(self, event):
		tc = event.GetEventObject()
		bak = s = tc.GetValue()
		nm = self.name_get(tc) # text_ctrl_ip_a_0
		t = nm[-3:-2] # a
		if s.isdigit():
			i = int(s)
			i = 0 if i < 0 else i
			i = 255 if i > 255 else i
			s = str(i)
		else:
			s = ''
		if s != bak:
			tc.SetValue(s)
		self.update_button_conn_stat(t)

	def OnConn(self, event):
		b = event.GetEventObject()
		nm = self.name_get(b) # button_conn_a
		t = nm[-1:] # a
		if t == 'b': # tablet
			cmd = 'roslaunch runtime_manager ui_socket.launch'
			sock = self.launch_kill(True, cmd, None)
		else:
			ipaddr = '.'.join([ self.text_ip_get(t, s).GetValue() for s in ['0','1','2','3'] ])
			port = 12345
			sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sock.connect((ipaddr, port))
		setattr(self, 'sock_' + t, sock)

		b.Disable()
		self.text_ip_stat_set(t, False)
		self.obj_get('button_disconn_' + t).Enable()

	def OnDisconn(self, event):
		b = event.GetEventObject()
		nm = self.name_get(b) # button_disconn_a
		t = nm[-1:] # a
		sock = self.obj_get('sock_' + t)
		if sock:
			if t == 'b': # tablet
				self.launch_kill(False, 'dmy', sock)
			else:
				sock.close()
			setattr(self, 'sock_' + t, None)
		b.Disable()
		self.text_ip_stat_set(t, True)
		self.update_button_conn_stat(t)

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

	def OnScAccel(self, event):
		self.OnMainSc(event)

	def OnScBrake(self, event):
		self.OnMainSc(event)

	def OnScSteer(self, event):
		self.OnMainSc(event)

	def OnMainSc(self, event):
		obj = event.GetEventObject()
		v = obj.GetValue()
		key = self.obj_key_get(obj, ['slider_statchk_'])
		msg = key + '_cmd'
		topic = '/' + msg
		klass_msg = globals()[msg]
		pub = rospy.Publisher(topic, klass_msg, queue_size=10)
		pub.publish( klass_msg(**{ key : v }) )

	def update_button_conn_stat(self, t): # a
		conn = self.obj_get('button_conn_' + t);
		en = conn.IsEnabled()
		if self.obj_get('sock_' + t) and en:
			conn.Disable()
			return
		yet = [ s for s in ['0','1','2','3'] if self.text_ip_get(t, s).GetValue() == '' ]
		act = None
		act = True if len(yet) <= 0 and not en else act
		act = False if len(yet) > 0 and en else act
		if act is not None:
			conn.Enable(act)

	def text_ip_get(self, t, s): # t a, s 0
		return self.obj_get('text_ctrl_ip_' + t + '_' + s)

	def text_ip_stat_set(self, t, en): # a
		for s in ['0','1','2','3']:
			self.text_ip_get(t, s).Enable(en)

	def radio_action(self, event, grp):
		push = event.GetEventObject()
		for b in grp:
			v = b.GetValue()
			act = None
			act = True if b is push and not v else act
			act = False if b is not push and v else act
			if act is not None:
				b.SetValue(act)

	def statchk_send_recv(self):
		#
		# send
		#
		sock = self.sock_c # Vehicle conn
		if sock is None: 
			print('Not connect !')
			return
		steer = self.slider_statchk_steer.GetValue()
		accel = self.slider_statchk_accel.GetValue()
		brake = self.slider_statchk_brake.GetValue()
		gear_dic = { 'b':0 , 'r':1 , 'n':2 , 'd':3 }
		gear = self.radio_value_get('button_statchk_', gear_dic)
		mode_dic = { 'prog':0, 'manu':1 }
		mode = self.radio_value_get('button_statchk_', mode_dic)
		data = struct.pack('=5i', steer, accel, brake, gear, mode)
		sock.send(data)

		#
		# recv
		#
		rdata = sock.recv(1024)
		(r_steer, r_accel, r_brake, r_gear, r_mode) = struct.unpack('=5i', rdata)
		
		self.radio_value_set('button_statchk_', gear_dic, r_gear)
		self.radio_value_set('button_statchk_', mode_dic, r_mode)
		self.slider_statchk_steer.SetValue(r_steer)
		self.slider_statchk_accel.SetValue(r_accel)
		self.slider_statchk_brake.SetValue(r_brake)

		s = self.key_get(gear_dic, r_gear)
		self.label_gear.SetLabel(s.upper() if s else '?')
		s = self.key_get(mode_dic, r_mode)
		self.label_mode.SetLabel(s[0].upper() if s else '?')

	def radio_value_get(self, base_name, dic):
		return get_top( [ v for (s,v) in dic.items() if self.obj_get(base_name + s).GetValue() ], 0 )

	def radio_value_set(self, base_name, dic, val):
		for (k,v) in dic.items():
			obj = self.obj_get(base_name + k)
			ov = obj.GetValue()
			act = None
			act = True if v == val and not ov else act
			act = False if v != val and ov else act
			if act is not None:
				obj.SetValue(act)

	#
	# Computing Tab
	#
	def OnTreeChecked(self, event):
		self.launch_kill_proc(event.GetItem(), self.computing_cmd)

	def OnTreeHyperlinked(self, event):
		item = event.GetItem()		
		info = self.config_dic.get(item, None)
		if info is None:
			return
		pdic = info['pdic']
		prm = self.get_param(info['param'])
		dlg = MyDialogParam(self, pdic=pdic, prm=prm)
		dlg.ShowModal()

	def publish_param_topic(self, pdic, prm):
		pub = prm['pub']
		klass_msg = globals()[ prm['msg'] ]
		msg = klass_msg(**pdic)

		for name in msg.__slots__[1:]: # skip 'header'
			type_str = msg._slot_types[ msg.__slots__.index(name) ]
			s = getattr(msg, name)
			setattr(msg, name, str_to_rosval(s, type_str, s))

		pub.publish(msg)

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
		self.launch_kill_proc(event.GetEventObject(), self.sensing_cmd)

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
			if 'path' in dic:
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
		self.add_cfg_info(cfg_obj, obj, name, pdic, True, 'path', dic['path'])
		return hszr

	#
	# Simulation Tab
	#
	def OnSimulation(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.simulation_cmd)

	def OnSimTime(self, event):
		obj = event.GetEventObject()
		cmd_dic = self.simulation_cmd
		(cmd, proc) = cmd_dic.get(obj, (None, None));
		if cmd and type(cmd) is dict:
			cmd = cmd.get(obj.GetValue(), None)
		if cmd:
			print(cmd)
			os.system(cmd)

	#
	# Data Tab
	#
	def OnTextArea(self, event):
		pf = 'text_ctrl_moving_objects_route_'
		lst = [ 'to_lat', 'to_lon', 'from_lat', 'from_lon' ]
		yet = [ nm for nm in lst if self.obj_get(pf + nm).GetValue() == '' ]
		en = len(yet) <= 0
		btn = self.button_launch_download
		if btn.IsEnabled() is not en:
			btn.Enable(en)

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
	def OnConfig(self, event):
		cfg_obj = event.GetEventObject()
		info = self.config_dic.get(cfg_obj, None)
		if info is None:
			return
		pdic = info['pdic']
		path_name = info['path']
		dlg = MyDialogPath(self, pdic=pdic, path_name=path_name)
		dlg.ShowModal()

	def get_cfg_obj(self, obj):
		return get_top( [ k for (k,v) in self.config_dic.items() if v['obj'] is obj ] )

	def add_cfg_info(self, cfg_obj, obj, name, pdic, run_disable, key, value):
		self.config_dic[ cfg_obj ] = { 'obj':obj , 'name':name , 'pdic':pdic , 'run_disable':run_disable , key:value }

	def get_cfg_info(self, obj):
		cfg_obj = self.get_cfg_obj(obj)
		return self.config_dic[ cfg_obj ] if cfg_obj else None

	def get_cfg_pdic(self, obj):
		info = self.get_cfg_info(obj)
		return info[ 'pdic' ] if info else None

	def get_param(self, prm_name):
		return get_top( [ prm for prm in self.params if prm['name'] == prm_name ] )

	def setup_config_param_pdic(self):
		for info in self.config_dic.values():
			if 'param' in info and info['pdic'] is None:
				self.setup_create_pdic(info)

	def setup_create_pdic(self, targ_info):
		prm_name = targ_info['param']
		info = get_top( [ info for info in self.config_dic.values() if info.get('param', None) == prm_name and info['pdic'] ] )
		if info:
			targ_info['pdic'] = info['pdic']
			return
		pdic = {}
		prm = self.get_param(prm_name)
		if prm:
			for var in prm['vars']:
				pdic[ var['name'] ] = var['v']
		targ_info['pdic'] = pdic
		self.load_dic[ targ_info['name'] ] = pdic

	def get_cmd_dic(self, key):
		dic = { 'tf'		: self.sensing_cmd,
			'sensor_fusion'	: self.sensing_cmd,
			'rosbag_play'	: self.simulation_cmd,
			'pmap'		: self.simulation_cmd,
			'vmap'		: self.simulation_cmd,
			'trajectory'	: self.simulation_cmd,
			'download'	: self.data_cmd,
			'upload'	: self.data_cmd,
		}
		return dic.get(key, None)

	def OnLaunch(self, event):
		obj = event.GetEventObject()
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

		cmd_dic = self.get_cmd_dic(key)
		if obj not in cmd_dic:
			return
		(cmd, proc) = cmd_dic[obj]

		add_args = None
		if path:
			# Vector Map default setting
			names = self.vmap_names if key == 'vmap' else None
			add_args = [ path + '/' + nm for nm in names ] if names else path.split(',')

		if key == 'rosbag_play':
			rate = self.val_get('text_ctrl_rate_' + key)
			if rate and rate is not '':
				add_args = [ '-r', rate ] + ( add_args if add_args else [] )
			if self.val_get('checkbox_clock_' + key):
				add_args = [ '--clock' ] + ( add_args if add_args else [] )

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
		key = self.obj_key_get(kill_obj, ['button_kill_'])
		if not key:
			return
		obj = self.obj_get('button_launch_' + key)
		cmd_dic = self.get_cmd_dic(key)
		if obj not in cmd_dic:
			return
		(cmd, proc) = cmd_dic[obj]

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
		key = self.obj_key_get(pause_obj, ['button_pause_'])
		if not key:
			return
		obj = self.obj_get('button_launch_' + key)
		cmd_dic = self.get_cmd_dic(key)
		if obj not in cmd_dic:
			return
		(cmd, proc) = cmd_dic[obj]
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
				self.add_config_link_tree_item(item, name, items['param'])

		for sub in items.get('subs', []):
			self.create_tree(parent, sub, tree, item, cmd_dic)
		return tree

	def add_config_link_tree_item(self, item, name, prm_name):
		pdic = self.load_dic.get(name, None)
		self.add_cfg_info(item, item, name, pdic, False, 'param', prm_name)
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

		info = self.get_cfg_info(obj)
		pdic = self.get_cfg_pdic(obj)
		if pdic and 'path' in info:
			add_args = [] if add_args is None else add_args
			add_args += [ k + ":=" + str(val) for (k,val) in pdic.items() ]

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

class MyDialogPath(rtmgr.MyDialogPath):
	def __init__(self, *args, **kwds):
		self.pdic = kwds.pop('pdic')
		self.path_name = kwds.pop('path_name')
		rtmgr.MyDialogPath.__init__(self, *args, **kwds)

		self.SetTitle(self.path_name)
		path = self.pdic.get(self.path_name, '')
		self.text_ctrl.SetValue(path)

	def OnRef(self, event):
		tc = self.text_ctrl
		path = tc.GetValue()
		(dn, fn) = os.path.split(path)
		dlg = wx.FileDialog(self, defaultDir=dn, defaultFile=fn)
		if dlg.ShowModal() == wx.ID_OK:
			tc.SetValue(dlg.GetPath())
			tc.SetInsertionPointEnd()
		dlg.Destroy()

	def OnOk(self, event):
		self.pdic[self.path_name] = str(self.text_ctrl.GetValue())
		self.EndModal(0)

	def OnCancel(self, event):
		self.EndModal(-1)

class VarPanel(wx.Panel):
	def __init__(self, *args, **kwds):
		self.var = kwds.pop('var')
		v = kwds.pop('v')
		wx.Panel.__init__(self, *args, **kwds)

		self.min = self.var.get('min', None)
		self.max = self.var.get('max', None)
		self.has_slider = self.min is not None and self.max is not None

		self.kind = self.var.get('kind', None)
		if self.kind == 'radio_box':
			label = self.var.get('label', '')
			choices = self.var.get('choices', [])
			self.obj = wx.RadioBox(self, wx.ID_ANY, label, choices=choices, majorDimension=0, style=wx.RA_SPECIFY_ROWS)
			self.obj.SetSelection(v)
			return

		szr = wx.BoxSizer(wx.HORIZONTAL)

		lb = wx.StaticText(self, wx.ID_ANY, self.var['label'])
		flag = wx.TOP | wx.BOTTOM | wx.LEFT | wx.ALIGN_CENTER_VERTICAL
		szr.Add(lb, 0, flag, 4)

		self.tc = wx.TextCtrl(self, wx.ID_ANY, str(v), style=wx.TE_PROCESS_ENTER)
		self.Bind(wx.EVT_TEXT_ENTER, self.OnTextEnter, self.tc)

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
		szr.Add(self.tc, 0, flag, 4)
		self.SetSizer(szr)

	def get_v(self):
		if self.kind == 'radio_box':
			return self.obj.GetSelection()
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

		panel_v = self.GetParent()
		dlg = panel_v.GetParent()
		dlg.update_pdic()
		dlg.publish()

	def OnTextEnter(self, event):
		if self.has_slider:
			self.slider.SetValue(self.get_int_v())

class MyDialogParam(rtmgr.MyDialogParam):
	def __init__(self, *args, **kwds):
		self.pdic = kwds.pop('pdic')
		self.pdic_bak = self.pdic.copy()
		self.prm = kwds.pop('prm')
		rtmgr.MyDialogParam.__init__(self, *args, **kwds)

		hszr = None
		self.vps = []
		for var in self.prm['vars']:
			v = self.pdic[ var['name'] ]
			vp = VarPanel(self.panel_v, var=var, v=v)
			if vp.has_slider:
				hszr = None if hszr else hszr
				self.sizer_v.Add(vp, 0, wx.EXPAND)
			else:
				if hszr is None:
					hszr = wx.BoxSizer(wx.HORIZONTAL)
					self.sizer_v.Add(hszr, 0, wx.EXPAND)
				hszr.Add(vp, 0, 0)
			self.vps.append(vp)

		self.SetTitle(self.prm['name'])
		(w,h) = self.GetSize()
		(w2,_) = self.sizer_v.GetMinSize()
		w2 += 20
		if w2 > w:
			self.SetSize((w2,h))

	def OnOk(self, event):
		self.update_pdic()
		self.publish()
		self.EndModal(0)

	def OnCancel(self, event):
		self.pdic.update(self.pdic_bak) # restore
		self.publish()
		self.EndModal(-1)

	def update_pdic(self):
		vars = self.prm['vars']
		for var in vars:
			v = self.vps[ vars.index(var) ].get_v()
			self.pdic[ var['name'] ] = v

	def publish(self):
		if 'pub' in self.prm:
			frame = self.GetParent()
			frame.publish_param_topic(self.pdic, self.prm)

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
		lst = subprocess.check_output([ 'rostopic', 'list' ]).split('\n')
		lst = [ 'All' ] + lst[:-1] # add All , remove last ''
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
