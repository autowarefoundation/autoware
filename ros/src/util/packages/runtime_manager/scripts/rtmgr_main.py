#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl
import gettext
import os
import socket
import struct
import shlex
import subprocess
import rtmgr

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)

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

		# for Sensing Tab
		self.drivers_cmd = {
			self.checkbox_camera_pggh3_usb1 : ('roslaunch pointgrey grasshopper3.launch', None),
			self.checkbox_camera_pggh3_usb2 : ('', None),
			self.checkbox_camera_pglb5 : ('', None),
			self.checkbox_camera_usb_gen : ('rosrun uvc_camera uvc_camera_node', None),
			self.checkbox_camera_ieee1394 : ('', None),
			self.checkbox_gnss_javad_d3_tty1 : ('roslaunch javad nmea_navsat.launch', None),
			self.checkbox_imu_crossbow_vg440 : ('', None),
			self.checkbox_lidars_velodyne_hdl_64e : ('roslaunch velodyne velodyne_hdl64e.launch', None),
			self.checkbox_lidars_velodyne_hdl_32e : ('roslaunch velodyne velodyne_hdl32e.launch', None),
			self.checkbox_lidars_hokuyo_utm30lx_usb1 : ('roslaunch hokuyo hokuyo_utm30lx.launch', None),
			self.checkbox_lidars_hokuyo_utm30lx_usb2 : ('', None),
			self.checkbox_lidars_sick_lms5511 : ('', None),
			self.checkbox_lidars_ibeo_8l_single : ('', None),
			self.checkbox_other_sensors_xxxx_tty1 : ('rosrun turtlesim turtlesim_node', None), # for debug ...
		}
		self.etc_cmd = {
			self.checkbox_sensor_fusion : ('', None),
			self.checkbox_rosbag : ('', None),
			self.button_calibration : ('', None),
			self.button_tf : ('', None),
			self.button_rviz : ('rosrun rviz rviz', None),
		}

	def __do_layout(self):
		pass

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

	#
	# Main Tab
	#
	def OnStart(self, event):
		print "start!"

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
		self.statchk_send()

	def OnProgManu(self, event):
		grp = [ self.button_statchk_prog,
			self.button_statchk_manu ]
		self.radio_action(event, grp)
		self.statchk_send()

	def OnScAccel(self, event):
		self.statchk_send()

	def OnScBrake(self, event):
		self.statchk_send()

	def OnScSteer(self, event):
		self.statchk_send()

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

	def statchk_send(self):
		sock = self.sock_c # Vehicle conn
		if sock is None: 
			print 'Not connect !'
			return
		steer = self.slider_statchk_steer.GetValue()
		accel = self.slider_statchk_accel.GetValue()
		brake = self.slider_statchk_brake.GetValue()
		gear = self.radio_value_get('button_statchk_', { 'b':0 , 'r':1 , 'n':2 , 'd':3 })
		prog_manu = self.radio_value_get('button_statchk_', { 'prog':0, 'manu':1 })
		data = struct.pack('=5i', steer, accel, brake, gear, prog_manu)
		sock.send(data)

	def radio_value_get(self, base_name, dic):
		res = [ v for (s,v) in dic.items() if getattr(self, base_name + s).GetValue() ]
                return res[0] if len(res) > 0 else 0

	#
	# Sensing Tab
	#
	def OnSensingDriver(self, event):
		self.launch_kill_proc(event, self.drivers_cmd)

	def OnSensorFusion(self, event):
		self.launch_kill_proc(event, self.etc_cmd)

	def OnRosbag(self, event):
		self.launch_kill_proc(event, self.etc_cmd)
		
	def OnCalib(self, event):
		self.launch_kill_proc(event, self.etc_cmd)

	def OnTf(self, event):
		self.launch_kill_proc(event, self.etc_cmd)

	def OnRviz(self, event):
		self.launch_kill_proc(event, self.etc_cmd)

	def launch_kill_proc(self, event, cmd_dic):
		obj = event.GetEventObject()
		v = obj.GetValue()
		(cmd, proc) = cmd_dic[obj]
		if cmd == '':
			obj.SetValue(False)
                msg = None
		msg = 'already launched.' if v and proc else msg
		msg = 'already terminated.' if not v and proc is None else msg
		msg = 'cmd not implemented.' if cmd == '' else msg
		if msg is not None:
                        print msg
			return
		if v:
			args = shlex.split(cmd)
			print args
			proc = subprocess.Popen(args)
		else:
			proc.terminate()
			proc.wait()
			proc = None
		cmd_dic[obj] = (cmd, proc)

	#
	# Common Utils
	#
	def name_get(self, obj):
		nms = [ nm for nm in dir(self) if getattr(self, nm) is obj ]
		return nms[0] if len(nms) > 0 else None

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
