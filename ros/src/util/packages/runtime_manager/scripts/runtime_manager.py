#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl as CT
import gettext
import os
import socket
import struct
import shlex
import subprocess
import psutil
import yaml
import rtmgr
import rospy
import std_msgs.msg

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)

		#
		# ros
		#
		rospy.init_node('runime_manager', anonymous=True)
		rospy.Subscriber('to_rtmgr', std_msgs.msg.String, self.RosCb)
		self.pub = rospy.Publisher('from_rtmgr', std_msgs.msg.String, queue_size=10)

		#
		# for Version tab
		#
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		self.bitmap_1 = wx.StaticBitmap(self.tab_version, wx.ID_ANY, wx.Bitmap(dir + "nagoya_university.png", wx.BITMAP_TYPE_ANY))
		self.bitmap_2 = wx.StaticBitmap(self.tab_version, wx.ID_ANY, wx.Bitmap(dir + "axe.png", wx.BITMAP_TYPE_ANY))

		#
		# for Computing tab
		#
		parent = self.tree_ctrl.GetParent()
		self.tree_ctrl.Destroy()
		items = self.load_yaml('computing_launch_cmd.yaml')
		self.computing_cmd = {}
		self.tree_ctrl = self.create_tree(parent, items, None, None, self.computing_cmd)
		self.tree_ctrl.ExpandAll()
		self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)

		self.compu_viewer_cmd = {}
		for (k, v) in self.load_yaml('computing_viewer.yaml').items():
			obj = self.obj_get('button_' + k)
			if not obj:
				print ('not found button_' + k)
				continue
			if not v or 'name' not in v or 'cmd' not in v:
				continue
			obj.SetLabel(v['name'])
			obj.Show()
			self.compu_viewer_cmd[obj] = (v['cmd'], None)

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
		(self.drv_probe_cmd, self.sensing_cmd) = self.load_yaml_probe_run('sensing_cmd.yaml')

		# for button_fusion
		tc = self.text_ctrl_calibration
		path = os.path.expanduser("~") + '/.ros/autoware'
		tc.SetValue(path)
		tc.SetInsertionPointEnd()
		self.text_ctrl_fusion = tc
		self.button_ref_fusion = self.button_ref_calibration

		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.OnProbe, self.timer)
		self.probe_interval = 10*1000
		if self.checkbox_auto_probe.GetValue():
			self.OnProbe(None)
			self.timer.Start(self.probe_interval)

		#
		# for Simulation Tab
		#
		self.simulation_cmd = self.load_yaml_dic('simulation_launch_cmd.yaml')
		self.vmap_names = self.load_yaml('vector_map_files.yaml')

		self.sel_multi_ks = [ 'pmap' ]
		self.sel_dir_ks = [ 'vmap', 'calibration' ]

		self.text_ctrl_rviz_simu.Disable()
		self.button_ref_rviz_simu.Disable()

	def __do_layout(self):
		pass

	def RosCb(self, data):
		print('recv topic msg : ' + data.data)

		r = rospy.Rate(10)
		rospy.is_shutdown()
		r.sleep()
		self.pub.publish(data.data)
		r.sleep()

	def load_yaml_dic(self, filename):
		d = self.load_yaml(filename)
		ret_dic = {}
		for (k,v) in d.items():
			res = [ pfix for pfix in ['checkbox_','button_'] if self.obj_get(pfix + k) ]
			if len(res) <= 0:
				print(k + ' in file ' + filename + ', not found correspoinding widget.')
				continue
			obj = self.obj_get(res[0] + k)
			ret_dic[obj] = (v, None)
		return ret_dic

	def load_yaml_probe_run(self, filename):
		d = self.load_yaml(filename)
		probe_dic = {}
		run_dic = {}
		for (k,d2) in d.items():
			res = [ pfix for pfix in ['checkbox_','button_'] if self.obj_get(pfix + k) ]
			if len(res) <= 0:
				print(k + ' in file ' + filename + ', not found correspoinding widget.')
				continue
			obj = self.obj_get(res[0] + k)
			if not d2 or type(d2) is not dict:
				continue
			if 'probe' in d2:
				probe_dic[obj] = (d2['probe'], None)
			if 'run' in d2:
                                run_dic[obj] = (d2['run'], None)
		return (probe_dic, run_dic)

	#
	# Main Tab
	#
	def OnStart(self, event):
		print("start!")

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
			sock.close()
			setattr(self, 'sock_' + t, None)
		b.Disable()
		self.text_ip_stat_set(t, True)
		self.update_button_conn_stat(t)

	def OnGear(self, event):
		grp = [ self.button_statchk_d,
			self.button_statchk_r,
			self.button_statchk_b,
			self.button_statchk_n ]
		self.radio_action(event, grp)
		self.statchk_send_recv()

	def OnProgManu(self, event):
		grp = [ self.button_statchk_prog,
			self.button_statchk_manu ]
		self.radio_action(event, grp)
		self.statchk_send_recv()

	def OnScAccel(self, event):
		self.statchk_send_recv()

	def OnScBrake(self, event):
		self.statchk_send_recv()

	def OnScSteer(self, event):
		self.statchk_send_recv()

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
		res = [ v for (s,v) in dic.items() if self.obj_get(base_name + s).GetValue() ]
		return res[0] if len(res) > 0 else 0

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

	def OnCompuViewer(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.compu_viewer_cmd)

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.sensing_cmd)

	def OnFusion(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.sensing_cmd)

	def OnRosbag(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.sensing_cmd)
		
	def OnCalib(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.sensing_cmd)

	def OnTf(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.sensing_cmd)

	def OnRviz(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.sensing_cmd)

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
			en = obj.IsShown()
			if res and not en:
				obj.Show()
				continue
			if not res and en:
				v = obj.GetValue()
				if v:
					obj.SetValue(False)	
					self.launch_kill_proc(obj)
				obj.Hide()

	#
	# Simulation Tab
	#
	def OnRvizSimu(self, event):
		self.launch_kill_proc(event.GetEventObject(), self.simulation_cmd)

	def OnRosbagSimu(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.simulation_cmd)

	def OnPmap(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.simulation_cmd)

	def OnVmap(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.simulation_cmd, names=self.vmap_names)

	def OnMobility(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.simulation_cmd)

	def OnTrajectory(self, event):
		self.launch_kill_proc_file(event.GetEventObject(), self.simulation_cmd)

	#
	# Common Utils
	#
	def OnRef(self, event):
		b = event.GetEventObject()
		nm = self.name_get(b) # button_ref_xxx
		k = nm[ len('button_ref_'): ] # xxx
		tc = self.obj_get('text_ctrl_' + k)
		if tc is None:
			return
		path = tc.GetValue()
		multi = k in self.sel_multi_ks
		if k in self.sel_dir_ks:
			dlg = wx.DirDialog(self, defaultPath=path);
		else:
			(dn, fn) = os.path.split(path)
			style = wx.FD_MULTIPLE if multi else wx.FD_DEFAULT_STYLE 
			dlg = wx.FileDialog(self, defaultDir=dn, defaultFile=fn, style=style);
		if dlg.ShowModal() == wx.ID_OK:
			path = ','.join(dlg.GetPaths()) if multi else dlg.GetPath()
			tc.SetValue(path)
			tc.SetInsertionPointEnd()
		dlg.Destroy()

	def create_tree(self, parent, items, tree, item, cmd_dic):
		name = items['name'] if 'name' in items else ''
		if tree is None:
			style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
			tree = CT.CustomTreeCtrl(parent, wx.ID_ANY, style=style)
			item = tree.AddRoot(name)
		else:
			ct_type = 1 if 'cmd' in items else 0 # 1:checkbox type
			item = tree.AppendItem(item, name, ct_type=ct_type)
			if 'cmd' in items:
				cmd_dic[item] = (items['cmd'], None)
		for sub in items['subs'] if 'subs' in items else []:
			self.create_tree(parent, sub, tree, item, cmd_dic)
		return tree

	def launch_kill_proc_file(self, obj, cmd_dic, names=None):
		name = self.name_get(obj)
		pfs = [ 'button_', 'checkbox_' ]
		keys = [ name[len(pf):] for pf in pfs if name[0:len(pf)] == pf ]
		if len(keys) <= 0:
			return
		key = keys[0]
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
			obj.SetValue(False)
			print('not implemented.')
			return
		v = obj.GetValue()
		(cmd, proc) = cmd_dic[obj]
		if not cmd:
			obj.SetValue(False)
		msg = None
		msg = 'already launched.' if v and proc else msg
		msg = 'already terminated.' if not v and proc is None else msg
		msg = 'cmd not implemented.' if not cmd else msg
		if msg is not None:
			print(msg)
			return
		if v:
			t = cmd
			if type(t) is list:
				t = self.modal_dialog(obj, t)
				if t is None:
					return # cancel
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
			proc = subprocess.Popen(args)
		else:
			terminate_children(proc.pid)
			proc.terminate()
			proc.wait()
			proc = None
		cmd_dic[obj] = (cmd, proc)

	def modal_dialog(self, obj, lst):
		(lbs, cmds) = zip(*lst)
		dlg = MyDialog(self, lbs=lbs)
		dlg.SetTitle(obj.GetLabel())
		r = dlg.ShowModal()
		ok = (0 <= r and r < len(cmds))
		if not ok:
			obj.SetValue(False)
		return cmds[r] if ok else None

	def load_yaml(self, filename):
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		f = open(dir + filename, 'r')
		d = yaml.load(f)
		f.close()
		return d

	def name_get(self, obj):
		nms = [ nm for nm in dir(self) if getattr(self, nm) is obj ]
		return nms[0] if len(nms) > 0 else None

	def obj_get(self, name):
		return getattr(self, name, None)

	def key_get(self, dic, val):
		ks = [ k for (k,v) in dic.items() if v == val ]
		return ks[0] if len(ks) > 0 else None

class MyDialog(rtmgr.MyDialog):
	def __init__(self, *args, **kwds):
		lbs = kwds['lbs']
		del kwds['lbs']
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

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = MyFrame(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1

def terminate_children(pid):
	for child in psutil.Process(pid).get_children():
		child.terminate()

def prn_dict(dic):
	for (k,v) in dic.items():
		print (k, ':', v)

if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()

# EOF
