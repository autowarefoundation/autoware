#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl
import gettext
import os
import socket
import struct
import shlex
import subprocess
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

		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		self.bitmap_1 = wx.StaticBitmap(self.tab_version, wx.ID_ANY, wx.Bitmap(dir + "nagoya_university.png", wx.BITMAP_TYPE_ANY))
		self.bitmap_2 = wx.StaticBitmap(self.tab_version, wx.ID_ANY, wx.Bitmap(dir + "axe.png", wx.BITMAP_TYPE_ANY))

		tab_nodes = self.notebook_1_pane_3

		self.tree_ctrl_1.Destroy()
		items = []
		self.tree_ctrl_1 = self.create_tree(tab_nodes, items, 'Actuation')

		self.tree_ctrl_2.Destroy()
		items = []
		self.tree_ctrl_2 = self.create_tree(tab_nodes, items, 'Sensing')

		self.tree_ctrl_3.Destroy()

		items = [
		    [ 'contorl' ],
		    [ 'perception', [
			[ 'detection', [
			    [ 'car_detector', True ],
			    [ 'hog_extractor', False ],
			    [ 'lane_detecto', True ],
			    [ 'pedestrian_detecto', False ] ] ] ] ],
		    [ 'planning' ] ]

		self.tree_ctrl_3 = self.create_tree(tab_nodes, items, 'Computing')

		rtmgr.MyFrame.__do_layout(self);

		# for Main Tab
		self.sock_a = None
		self.sock_b = None
		self.sock_c = None
		self.sock_d = None

		#
		# for Sensing Tab
		#
		self.drv_probe_cmd = self.load_yaml('drivers_probe_cmd.yaml')
		self.sensing_cmd = self.load_yaml('sensing_launch_cmd.yaml')

		self.timer = wx.Timer(self)
		self.Bind(wx.EVT_TIMER, self.OnProbeTimer, self.timer)
		self.OnProbeTimer(None)
		self.timer.Start(10*1000)

	def __do_layout(self):
		pass

	def RosCb(self, data):
		print('recv topic msg : ' + data.data)

		r = rospy.Rate(10)
		rospy.is_shutdown()
		r.sleep()
		self.pub.publish(data.data)
		r.sleep()

	def create_tree(self, parent, items, root_name):
		style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
		tree = wx.lib.agw.customtreectrl.CustomTreeCtrl(parent, wx.ID_ANY, style=style)
		root = tree.AddRoot(root_name)
		self.append_items(tree, root, items)
		tree.ExpandAll()
		return tree

	def append_items(self, tree, item, items):
		for add in items:
			ct_type = 1 if len(add) > 1 and type(add[1]) is bool else 0
			add_item = tree.AppendItem(item, add[0], ct_type=ct_type)
			if len(add) > 1:
				if type(add[1]) is bool:
					if add[1] is True:
						add_item.Set3StateValue(wx.CHK_CHECKED)
				else:
					self.append_items(tree, add_item, add[1])

	def load_yaml(self, filename):
		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		f = open(dir + filename, 'r')
		d = yaml.load(f)
		f.close()

		ret_dic = {}
		for (k,v) in d.items():
			res = [ pfix for pfix in ['checkbox_','button_'] if self.obj_get(pfix + k) ]
			if len(res) <= 0:
				print(k + ' in file ' + filename + ', not found correspoinding widget.')
				continue
			obj = self.obj_get(res[0] + k)
			ret_dic[obj] = (v, None)
		return ret_dic

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
		getattr(self, 'button_disconn_' + t).Enable()

	def OnDisconn(self, event):
		b = event.GetEventObject()
		nm = self.name_get(b) # button_disconn_a
		t = nm[-1:] # a
		sock = getattr(self, 'sock_' + t)
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
		conn = getattr(self, 'button_conn_' + t);
		en = conn.IsEnabled()
		if getattr(self, 'sock_' + t) and en:
			conn.Disable()
			return
		yet = [ s for s in ['0','1','2','3'] if self.text_ip_get(t, s).GetValue() == '' ]
		act = None
		act = True if len(yet) <= 0 and not en else act
		act = False if len(yet) > 0 and en else act
		if act is not None:
			conn.Enable(act)

	def text_ip_get(self, t, s): # t a, s 0
		return getattr(self, 'text_ctrl_ip_' + t + '_' + s)

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
		res = [ v for (s,v) in dic.items() if getattr(self, base_name + s).GetValue() ]
                return res[0] if len(res) > 0 else 0

	def radio_value_set(self, base_name, dic, val):
		for (k,v) in dic.items():
			obj = getattr(self, base_name + k)
			ov = obj.GetValue()
			act = None
			act = True if v == val and not ov else act
			act = False if v != val and ov else act
			if act is not None:
				obj.SetValue(act)

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.launch_kill_proc(event.GetEventObject())

	def OnSensorFusion(self, event):
		self.launch_kill_proc(event.GetEventObject())

	def OnRosbag(self, event):
		self.launch_kill_proc(event.GetEventObject())
		
	def OnCalib(self, event):
		self.launch_kill_proc(event.GetEventObject())

	def OnTf(self, event):
		self.launch_kill_proc(event.GetEventObject())

	def OnRviz(self, event):
		self.launch_kill_proc(event.GetEventObject())

	def launch_kill_proc(self, obj):
		cmd_dic = self.sensing_cmd
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
			args = shlex.split(cmd)
			print(args)
			proc = subprocess.Popen(args)
		else:
			proc.terminate()
			proc.wait()
			proc = None
		cmd_dic[obj] = (cmd, proc)

	def OnProbeTimer(self, event):
		#print('probe') # for debug
                items = self.drv_probe_cmd.items()
		for (obj, (cmd, bak_res)) in items:
			res = (os.system(cmd) == 0) if cmd else False
			if res == bak_res:
				continue
			self.drv_probe_cmd[obj] = (cmd, res)
			en = obj.IsEnabled()
			if res and not en:
				obj.Enable()
				continue
			if not res and en:
				v = obj.GetValue()
				if v:
					obj.SetValue(False)	
					self.launch_kill_proc(obj)
				obj.Disable()

	#
	# Common Utils
	#
	def name_get(self, obj):
		nms = [ nm for nm in dir(self) if getattr(self, nm) is obj ]
		return nms[0] if len(nms) > 0 else None

	def obj_get(self, name):
		return getattr(self, name) if name in dir(self) else None

	def key_get(self, dic, val):
		ks = [ k for (k,v) in dic.items() if v == val ]
		return ks[0] if len(ks) > 0 else None

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = MyFrame(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1

if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()

# EOF
