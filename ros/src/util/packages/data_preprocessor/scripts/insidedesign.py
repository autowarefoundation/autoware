#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import wx
import gettext
from baseframe import BaseFrame

class InsideDesign(BaseFrame):
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        super(InsideDesign, self).__init__(*args, **kwds)
        self.notebook_1 = wx.Notebook(self, wx.ID_ANY, style=0)
        self.tab_simulation = wx.Panel(self.notebook_1, wx.ID_ANY)

        ####################
        ##   ref area
        ####################
        self.panel_rosbag_play = wx.Panel(self.tab_simulation, wx.ID_ANY)
        self.sizer_79_staticbox = wx.StaticBox(self.tab_simulation, wx.ID_ANY, "")
        self.button_confirm_topics = wx.Button(self.tab_simulation, wx.ID_ANY, _("Refresh"))
        self.button_rviz = wx.ToggleButton(self.tab_simulation, wx.ID_ANY, _("RViz"))


        ###########################
        ##    scroll module
        ###########################
        self.panel_5 = wx.ScrolledWindow(self.tab_simulation, wx.ID_ANY, style=wx.TAB_TRAVERSAL)
        self.label_rosbag_info = wx.StaticText(self.panel_5, wx.ID_ANY, "")
        self.bitmap_logo = wx.StaticBitmap(self, wx.ID_ANY, wx.NullBitmap)

        ###############################################
        ##   For Select Tab
        ###############################################
        self.tab_select = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.sizer_select_box = wx.BoxSizer(wx.VERTICAL)
        self.sizer_select_topics = wx.BoxSizer(wx.VERTICAL)
        self.select_scroll =  wx.ScrolledWindow(self.tab_select, wx.ID_ANY, style=wx.TAB_TRAVERSAL)

        ################################
        ##   For Depth Tab
        ################################
        self.tab_depth = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.sizer_depth_box = wx.BoxSizer(wx.VERTICAL)
        self.sizer_depth_topics = wx.BoxSizer(wx.VERTICAL)
        self.depth_scroll = wx.ScrolledWindow(self.tab_depth, wx.ID_ANY, style=wx.TAB_TRAVERSAL)
        # self.panel_rosbag_play2 = wx.Panel(self.tab_depth, wx.ID_ANY)
        ########################
        ## play stop pause bar area
        ########################
        self.button_play_rosbag_play = wx.ToggleButton(self.tab_select, wx.ID_ANY, _("Start Conversion"))
        self.button_stop_rosbag_play = wx.ToggleButton(self.tab_select, wx.ID_ANY, _("Stop"))
        self.button_pause_rosbag_play = wx.ToggleButton(self.tab_select, wx.ID_ANY, _("Pause"))
        self.label_rosbag_play_bar = wx.StaticText(self.tab_select, wx.ID_ANY, _("Playing ... 82 %"))
        self.label_rosbag_play_pos = wx.StaticText(self.tab_select, wx.ID_ANY, "")
        self.static_line_3 = wx.StaticLine(self.tab_select, wx.ID_ANY)
        self.label_rosbag_play_total = wx.StaticText(self.tab_select, wx.ID_ANY, "")
        self.button_stop_rosbag_play.Enable(False)
        self.button_stop_rosbag_play.SetValue(1)
        self.button_pause_rosbag_play.Enable(False)
        self.label_rosbag_play_pos.SetMinSize((32, 17))
        self.label_rosbag_play_total.SetMinSize((32, 17))

        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay, self.button_play_rosbag_play)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay, self.button_stop_rosbag_play)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay, self.button_pause_rosbag_play)

        self.button_confirm_depth = wx.ToggleButton(self.tab_depth, wx.ID_ANY, _("confirm"))
        self.button_play_rosbag_play2 = wx.ToggleButton(self.tab_depth, wx.ID_ANY, _("Start Conversion"))
        self.button_stop_rosbag_play2 = wx.ToggleButton(self.tab_depth, wx.ID_ANY, _("Stop"))
        self.button_pause_rosbag_play2 = wx.ToggleButton(self.tab_depth, wx.ID_ANY, _("Pause"))
        self.label_rosbag_play_bar2 = wx.StaticText(self.tab_depth, wx.ID_ANY, _("Playing ... 82 %"))
        self.label_rosbag_play_pos2 = wx.StaticText(self.tab_depth, wx.ID_ANY, "")
        self.static_line_2 = wx.StaticLine(self.tab_depth, wx.ID_ANY)
        self.label_rosbag_play_total2 = wx.StaticText(self.tab_depth, wx.ID_ANY, "")
        self.button_stop_rosbag_play2.Enable(False)
        self.button_stop_rosbag_play2.SetValue(1)
        self.button_pause_rosbag_play2.Enable(False)
        self.label_rosbag_play_pos2.SetMinSize((32, 17))
        self.label_rosbag_play_total2.SetMinSize((32, 17))

        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay2, self.button_play_rosbag_play2)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay2, self.button_stop_rosbag_play2)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnROSbagPlay2, self.button_pause_rosbag_play2)


        self.__set_properties()

        self.Bind(wx.EVT_BUTTON, self.OnGetConfirmTopics, self.button_confirm_topics)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnRviz, self.button_rviz)
        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnConfirmDepth, self.button_confirm_depth)

    def __set_properties(self):
        super(InsideDesign, self).set_properties()
        self.panel_5.SetScrollRate(10, 10)
        self.select_scroll.SetScrollRate(10, 10)
        self.depth_scroll.SetScrollRate(10, 10)

    def _do_layout(self):
        self.sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_29 = wx.BoxSizer(wx.HORIZONTAL)
        # self.sizer_cpuinfo = wx.BoxSizer(wx.HORIZONTAL)

        ######################3
        ##    大きい中の枠組みの定義
        #######################
        sizer_78 = wx.BoxSizer(wx.VERTICAL)
        sizer_37 = wx.BoxSizer(wx.HORIZONTAL)


        self.sizer_79_staticbox.Lower()
        sizer_79 = wx.StaticBoxSizer(self.sizer_79_staticbox, wx.VERTICAL)
        sizer_79.Add(self.panel_rosbag_play, 1, wx.ALL | wx.EXPAND, 4)
        # sizer_79.Add(self.panel_rosbag_play2, 1, wx.ALL | wx.EXPAND, 4)
        sizer_button_topics = wx.BoxSizer(wx.HORIZONTAL)
        sizer_button_topics.Add(self.button_confirm_topics, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_button_topics.Add(self.button_rviz, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_79.Add(sizer_button_topics)
        sizer_78.Add(sizer_79, 0, wx.ALL | wx.EXPAND, 4)

		################################3
		##   sizer_80  play, stop pause
		################################3
        sizer_play_stop_pause_bar_area = wx.BoxSizer(wx.HORIZONTAL)
        sizer_play_bar_area = wx.BoxSizer(wx.HORIZONTAL)
        sizer_inside_play_bar_area = wx.BoxSizer(wx.VERTICAL)
        sizer_inside_play_stop_pause = wx.BoxSizer(wx.HORIZONTAL)
        sizer_inside_play_stop_pause.Add(self.button_play_rosbag_play, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_inside_play_stop_pause.Add(self.button_stop_rosbag_play, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_inside_play_stop_pause.Add(self.button_pause_rosbag_play, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_play_stop_pause_bar_area.Add(sizer_inside_play_stop_pause, 1, wx.EXPAND, 0)
        sizer_play_bar_area.Add(self.label_rosbag_play_bar, 1, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_inside_play_bar_area.Add(self.label_rosbag_play_pos, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_inside_play_bar_area.Add(self.static_line_3, 0, wx.EXPAND, 0)
        sizer_inside_play_bar_area.Add(self.label_rosbag_play_total, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_play_bar_area.Add(sizer_inside_play_bar_area, 0, wx.ALL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_play_stop_pause_bar_area.Add(sizer_play_bar_area, 1, wx.EXPAND, 0)
        self.sizer_select_box.Add(self.select_scroll, 1, wx.EXPAND, 0)
        self.sizer_select_box.Add(sizer_play_stop_pause_bar_area, 0, wx.ALL | wx.EXPAND, 4)

        sizer_play_stop_pause_bar_area2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_play_bar_area2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_inside_play_bar_area2 = wx.BoxSizer(wx.VERTICAL)
        sizer_inside_play_stop_pause2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_inside_play_stop_pause2.Add(self.button_confirm_depth, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 2)
        sizer_inside_play_stop_pause2.Add(self.button_play_rosbag_play2, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 2)
        sizer_inside_play_stop_pause2.Add(self.button_stop_rosbag_play2, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 2)
        sizer_inside_play_stop_pause2.Add(self.button_pause_rosbag_play2, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 2)
        sizer_play_stop_pause_bar_area2.Add(sizer_inside_play_stop_pause2, 1, wx.EXPAND, 0)
        sizer_play_bar_area2.Add(self.label_rosbag_play_bar2, 1, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_inside_play_bar_area2.Add(self.label_rosbag_play_pos2, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_inside_play_bar_area2.Add(self.static_line_2, 0, wx.EXPAND, 0)
        sizer_inside_play_bar_area2.Add(self.label_rosbag_play_total2, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL, 0)
        sizer_play_bar_area2.Add(sizer_inside_play_bar_area2, 0, wx.ALL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL, 4)
        sizer_play_stop_pause_bar_area2.Add(sizer_play_bar_area2, 1, wx.EXPAND, 0)
        self.sizer_depth_box.Add(self.depth_scroll, 1, wx.EXPAND, 0)
        self.sizer_depth_box.Add(sizer_play_stop_pause_bar_area2, 0, wx.ALL | wx.EXPAND, 4)

        ###############################
        # only scroll window    self.panel_5
        ###############################
        sizer_37.Add(self.label_rosbag_info, 1, wx.ALL | wx.EXPAND, 4)
        self.panel_5.SetSizer(sizer_37)
        sizer_78.Add(self.panel_5, 1, wx.EXPAND, 0)


        self.tab_status = wx.Panel(self.notebook_1, wx.ID_ANY)
        self.panel_3 = wx.ScrolledWindow(self.tab_status, wx.ID_ANY, style=wx.TAB_TRAVERSAL)
        self.label_top_cmd = wx.StaticText(self.panel_3, wx.ID_ANY, "")


        self.tab_simulation.SetSizer(sizer_78)
        self.tab_select.SetSizer(self.sizer_select_box)
        self.tab_depth.SetSizer(self.sizer_depth_box)
        self.notebook_1.AddPage(self.tab_simulation, _("Input Bag File"))
        self.notebook_1.AddPage(self.tab_select, _("Save object from Topics"))
        self.notebook_1.AddPage(self.tab_depth, _("Save Depth from Image and Lidar"))
        self.sizer_1.Add(self.notebook_1, 1, wx.EXPAND | wx.ALL, 0)

        # sizer_29.Add((0, 100), 0, wx.EXPAND, 0) # width line
        # sizer_29.Add(self.sizer_cpuinfo, 1, wx.EXPAND, 0)
        # self.sizer_1.Add(sizer_29, 0, wx.EXPAND, 0)
        #
        # self.sizer_1.Add(self.bitmap_logo, 0, 0, 0)

        self.SetSizer(self.sizer_1)
        self.Layout()

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

    def OnROSbagPlay2(self, event):  # wxGlade: MyFrame.<event_handler>
    	print "Event handler 'OnROSbagPlay2' not implemented!"
    	event.Skip()

    def OnRviz(self, event):
        print(1)
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

    def OnGetConfirmTopics(self, event):
        print("Event handler 'OnGetConfirmTopics' not implemented!")
        event.Skip()

    def OnConfirmDepth(self, event):
        print("Event handler 'OnConfirmDepth' not implemented")
        event.Skip()

class MyDialogROSbagRecord(wx.Dialog):
	def __init__(self, *args, **kwds):
		# begin wxGlade: MyDialogROSbagRecord.__init__
		kwds["style"] = wx.DEFAULT_DIALOG_STYLE
		wx.Dialog.__init__(self, *args, **kwds)
		self.text_ctrl = wx.TextCtrl(self, wx.ID_ANY, "")
		self.button_ref = wx.Button(self, wx.ID_ANY, _("Ref"))
		self.checkbox_split = wx.CheckBox(self, wx.ID_ANY, _("split"))
		self.label_2 = wx.StaticText(self, wx.ID_ANY, _("size"))
		self.text_ctrl_size = wx.TextCtrl(self, wx.ID_ANY, "")
		self.label_2_copy = wx.StaticText(self, wx.ID_ANY, _("MB"))
		self.button_start = wx.Button(self, wx.ID_ANY, _("Start"))
		self.button_stop = wx.Button(self, wx.ID_ANY, _("Stop"))
		self.panel_1 = wx.ScrolledWindow(self, wx.ID_ANY, style=wx.TAB_TRAVERSAL)
		self.button_refresh = wx.Button(self, wx.ID_ANY, _("Refresh"))

		self.__set_properties()
		self.__do_layout()

		self.Bind(wx.EVT_BUTTON, self.OnRef, self.button_ref)
		self.Bind(wx.EVT_BUTTON, self.OnStart, self.button_start)
		self.Bind(wx.EVT_BUTTON, self.OnStop, self.button_stop)
		self.Bind(wx.EVT_BUTTON, self.OnRefresh, self.button_refresh)
		# end wxGlade

	def __set_properties(self):
		# begin wxGlade: MyDialogROSbagRecord.__set_properties
		self.SetTitle(_("ROSBAG Record"))
		self.SetSize((300, 430))
		self.button_ref.SetMinSize((40, 29))
		self.text_ctrl_size.SetMinSize((50, 27))
		self.button_stop.Enable(False)
		self.panel_1.SetScrollRate(10, 10)
		# end wxGlade

	def __do_layout(self):
		# begin wxGlade: MyDialogROSbagRecord.__do_layout
		sizer_41 = wx.BoxSizer(wx.VERTICAL)
		self.sizer_topic = wx.BoxSizer(wx.VERTICAL)
		sizer_44 = wx.BoxSizer(wx.HORIZONTAL)
		sizer_22 = wx.BoxSizer(wx.HORIZONTAL)
		sizer_23 = wx.BoxSizer(wx.HORIZONTAL)
		sizer_28_copy_1 = wx.BoxSizer(wx.HORIZONTAL)
		sizer_28_copy_1.Add(self.text_ctrl, 1, wx.LEFT | wx.TOP, 4)
		sizer_28_copy_1.Add(self.button_ref, 0, wx.LEFT | wx.RIGHT | wx.TOP, 4)
		sizer_41.Add(sizer_28_copy_1, 0, wx.EXPAND, 0)
		sizer_22.Add(self.checkbox_split, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_23.Add(self.label_2, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_23.Add(self.text_ctrl_size, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_23.Add(self.label_2_copy, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_22.Add(sizer_23, 1, wx.LEFT | wx.EXPAND, 20)
		sizer_41.Add(sizer_22, 0, wx.EXPAND, 0)
		sizer_44.Add(self.button_start, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_44.Add(self.button_stop, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		sizer_41.Add(sizer_44, 0, wx.EXPAND, 0)
		self.panel_1.SetSizer(self.sizer_topic)
		sizer_41.Add(self.panel_1, 1, wx.EXPAND, 0)
		sizer_41.Add(self.button_refresh, 0, wx.ALL | wx.ALIGN_CENTER_VERTICAL, 4)
		self.SetSizer(sizer_41)
		self.Layout()
		# end wxGlade

	def OnRef(self, event):  # wxGlade: MyDialogROSbagRecord.<event_handler>
		print "Event handler 'OnRef' not implemented!"
		event.Skip()

	def OnStart(self, event):  # wxGlade: MyDialogROSbagRecord.<event_handler>
		print "Event handler 'OnStart' not implemented!"
		event.Skip()

	def OnStop(self, event):  # wxGlade: MyDialogROSbagRecord.<event_handler>
		print "Event handler 'OnStop' not implemented!"
		event.Skip()

	def OnRefresh(self, event):  # wxGlade: MyDialogROSbagRecord.<event_handler>
		print "Event handler 'OnRefresh' not implemented!"
		event.Skip()

# end of class MyDialogCarPedestrian
class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = InsideDesign(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		frame_1.Show()
		return 1

# end of class MyApp

if __name__ == "__main__":
	gettext.install("app") # replace with the appropriate catalog name

	app = MyApp(0)
	app.MainLoop()
