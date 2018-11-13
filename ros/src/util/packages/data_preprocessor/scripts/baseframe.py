#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import wx
import gettext

class BaseFrame(wx.Frame):
	def __init__(self, *args, **kwds):
		kwds["style"] = wx.DEFAULT_FRAME_STYLE
		super(BaseFrame, self).__init__(*args, **kwds)

		self.set_properties()
		self.__do_layout()

	def set_properties(self):
		self.SetTitle(_("Data Preprocessor"))
		self.SetSize((806, 584))

	def __do_layout(self):
		pass

	def OnLaunchKill(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnLaunchKill' not implemented!"
		event.Skip()

	def OnSetupLocalizer(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnSetupLocalizer' not implemented!"
		event.Skip()

	def OnCalibrationPublisher(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnCalibrationPublisher' not implemented!"
		event.Skip()

	def OnLamp(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnLamp' not implemented!"
		event.Skip()

	def OnIndi(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnIndi' not implemented!"
		event.Skip()

	def OnGear(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnGear' not implemented!"
		event.Skip()

	def OnQuery(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnQuery' not implemented!"
		event.Skip()

	def OnROSbagPlay(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnROSbagPlay' not implemented!"
		event.Skip()

	def OnFtrace(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnFtrace' not implemented!"
		event.Skip()

	def OnEcho(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnEcho' not implemented!"
		event.Skip()

	def OnRefreshTopics(self, event):  # wxGlade: MyFrame.<event_handler>
		print "Event handler 'OnRefreshTopics' not implemented!"
		event.Skip()

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = BaseFrame(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1


if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()
