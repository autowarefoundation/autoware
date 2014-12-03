#!/usr/bin/env python

import wx
import wx.lib.agw.customtreectrl
import gettext
import os
import rtmgr

class MyFrame(rtmgr.MyFrame):
	def __init__(self, *args, **kwds):
		rtmgr.MyFrame.__init__(self, *args, **kwds)

		dir = os.path.abspath(os.path.dirname(__file__)) + "/"
		self.bitmap_1 = wx.StaticBitmap(self.notebook_1_pane_4, wx.ID_ANY, wx.Bitmap(dir + "nagoya_university.png", wx.BITMAP_TYPE_ANY))
		self.bitmap_2 = wx.StaticBitmap(self.notebook_1_pane_4, wx.ID_ANY, wx.Bitmap(dir + "axe.png", wx.BITMAP_TYPE_ANY))

		items = [
		    [ 'contorl' ],
		    [ 'perception', [
			[ 'detection', [
			    [ 'car_detector', True ],
			    [ 'hog_extractor', False ],
			    [ 'lane_detecto', True ],
			    [ 'pedestrian_detecto', False ] ] ] ] ],
		    [ 'planning' ] ]

		self.tree_ctrl_3 = self.create_tree(self.notebook_1_pane_3, items)

		rtmgr.MyFrame.__do_layout(self);

	def __do_layout(self):
		pass

	def create_tree(self, parent, items):
		style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
		tree = wx.lib.agw.customtreectrl.CustomTreeCtrl(parent, wx.ID_ANY, style=style)
		root = tree.AddRoot('root')
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
