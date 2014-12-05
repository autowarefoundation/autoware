#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl
import gettext
import os
import socket
import rtmgr

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)

		tab_nodes = self.notebook_1_pane_3
		tab_version = self.notebook_1_pane_5

		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		self.bitmap_1 = wx.StaticBitmap(tab_version, wx.ID_ANY, wx.Bitmap(dir + "nagoya_university.png", wx.BITMAP_TYPE_ANY))
		self.bitmap_2 = wx.StaticBitmap(tab_version, wx.ID_ANY, wx.Bitmap(dir + "axe.png", wx.BITMAP_TYPE_ANY))

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

		self.sock_a = None
		self.sock_b = None
		self.sock_c = None
		self.sock_d = None

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

	def OnStart(self, event):
		print "start!"

	def OnTextIp(self, event):
		tc = event.GetEventObject()
		bak = s = tc.GetValue()
                if s.isdigit():
			i = int(s)
			i = 0 if i < 0 else i
			i = 255 if i > 255 else i
			s = '%d' % i
		else:
			s = ''
		if s != bak:
			tc.SetValue(s)

		nm = self.name_get(tc) # text_ctrl_ip_a_0
		yet = [ s for s in ['0','1','2','3'] if getattr(self, nm[:-1] + s).GetValue() == '' ]
		t = nm[-3:-2] # a

		conn = getattr(self, 'button_conn_' + t);
		en = conn.IsEnabled()
		act = None
		act = True if len(yet) <= 0 and not en else act
		act = False if len(yet) > 0 and en else act
		if act is not None:
			comm.Enable(act)

	def OnConn(self, event):
		b = event.GetEventObject()
		nm = self.name_get(b) # button_conn_a
		t = nm[-1:] # a
		ipaddr = ''
		for s in ['0','1','2','3']:
			ipaddr += getattr(self, 'text_ctrl_ip_' + t + '_' + s) + '.'
		ipaddr = ipaddr[:-1]

		print ipaddr

		port = 12345
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((ipaddr, port))
		setattr(self, 'sock_' + t, sock)

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
                pass

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
