#!/usr/bin/env python

import wx
import wx.lib.buttons
import wx.lib.agw.customtreectrl as CT
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
from insidedesign import InsideDesign
import rtmgr
import rospy
import std_msgs.msg
from std_msgs.msg import Bool
from decimal import Decimal
from get_rosbaginfo import get_type_and_topic

SCHED_OTHER = 0
SCHED_FIFO = 1
SCHED_RR = 2
PROC_MANAGER_SOCK="/tmp/autoware_proc_manager"

class Final(InsideDesign):
    def __init__(self, *args, **kwds):
        super(Final, self).__init__(*args, **kwds)
        self.all_procs = []
        self.all_cmd_dics = []
        self.all_procs = []
        self.all_cmd_dics = []
        self.load_dic = self.load_yaml('param.yaml', def_ret={})
        self.config_dic = {}
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.params = []
        self.all_tabs = []
        self.all_th_infs = []
        self.log_que = Queue.Queue()
        self.log_que_stdout = Queue.Queue()
        self.log_que_stderr = Queue.Queue()
        self.log_que_show = Queue.Queue()
		#####################################
		## ros
		#####################################
        rospy.init_node('runime_manager', anonymous=True)
        rospy.Subscriber('to_rtmgr', std_msgs.msg.String, self.ROSCb)
        self.pub = rospy.Publisher('from_rtmgr', std_msgs.msg.String, queue_size=10)

		#######################################
		# for Select Topic & Excecution Tab
		#######################################
        self.label_rosbag_play_bar.Destroy()
        self.label_rosbag_play_bar = BarLabel(self.tab_select, '  Playing...  ')
        self.label_rosbag_play_bar.Enable(False)
        self.label_rosbag_play_bar2.Destroy()
        self.label_rosbag_play_bar2 = BarLabel(self.tab_depth, '  Playing...  ')
        self.label_rosbag_play_bar2.Enable(False)
        self.play = 0
        self.file_path = ""
        self.select = 0
        self.topic_type = None
        tab = self.tab_simulation
        self.all_tabs.append(tab)
        self.simulation_cmd = {}
        self.all_cmd_dics.append(self.simulation_cmd)
        dic = self.load_yaml('tab_input.yaml')
        self.add_params(dic.get('params', []))
        self.setup_buttons(dic.get('buttons'), self.simulation_cmd)
        self.proc = 0
        self.output_url = ""
        self.calib_url = ""
        self.depth_flag = False
        self.selected_img = {}
        self.selected_pcd = {}
        self.image_for_depth = None
        self.pointcloud_for_depth = None
        self.objx = False
        self.velodyne_button = False
        #
        # self.depth_cmd = {}
        # self.all_cmd_dics.append(self.depth_cmd)
        # dic = self.load_yaml('tab_depth.yaml')
        # self.add_params(dic.get('params', []))
        # self.setup_buttons(dic.get('buttons'), self.depth_cmd)

        btn = self.button_play_rosbag_play
        # setup for rosbag info
        gdic = self.obj_to_gdic(btn, {})
        gdic_v = dic_getset(gdic, 'file', {})
        gdic_v['update_hook'] = self.rosbag_info_hook

        tc = self.obj_to_varpanel_tc(btn, 'file')
        if tc:
            self.rosbag_info_hook( tc.GetValue() )
        else:
            print("Please Set Bag File")

        self.topic_and_type_list = None
        self.select_topic_delete_dic = {0:[], 1:[]}
        self.selected_topic_dic = {}
        self.select_created_topic = {}
        self.selected_topic_dic2 = {}
        self.runtime_dic = self.load_yaml('runtime.yaml')
        self.cmd_dic = {}

        try:
            self._do_layout()
        except Exception as e:
            print(e)

        cond = lambda s : s.startswith('tab_')
        self.tab_names = [ self.name_get_cond(tab, cond=cond, def_ret='').replace('tab_', '', 1) for tab in self.all_tabs ]
        #
        new_btn_grps = ( lambda btn_names, tab_names=self.tab_names :
        	[ [ self.obj_get('button_{}_{}'.format(bn, tn)) for tn in tab_names ] for bn in btn_names ] )

        self.alias_grps = new_btn_grps( ('rosbag', 'rviz', 'rqt') )

        # ################################
        # ##    For CPU Bar
        # ################################
        # toprc = os.path.expanduser('~/.toprc')
        # backup = os.path.expanduser('~/.toprc-autoware-backup')
        # self.toprc_setup(toprc, backup)
        #
        # cpu_ibls = [ InfoBarLabel(self, 'CPU'+str(i)) for i in range(get_cpu_count()) ]
        # sz = sizer_wrap(cpu_ibls, wx.HORIZONTAL, 1, wx.EXPAND, 0)
        # self.sizer_cpuinfo.Add(sz, 8, wx.ALL | wx.EXPAND, 4)
        #
        #
        # self.lb_top5 = []
        # for i in range(5):
        # 	lb = wx.StaticText(self, wx.ID_ANY, '')
        # 	change_font_point_by_rate(lb, 0.75)
        # 	self.lb_top5.append(lb)
        # line = wx.StaticLine(self, wx.ID_ANY)
        # ibl = InfoBarLabel(self, 'Memory', bar_orient=wx.HORIZONTAL)
        # szr = sizer_wrap(self.lb_top5 + [ line, ibl ], flag=wx.EXPAND | wx.FIXED_MINSIZE)
        # self.sizer_cpuinfo.Add(szr, 2, wx.ALL | wx.EXPAND, 4)
        #
        # self.status_dic = self.load_yaml('status.yaml')
        #
        # th_arg = { 'setting':self.status_dic.get('top_cmd_setting', {}),
        # 	   'cpu_ibls':cpu_ibls, 'mem_ibl':ibl,
        # 	   'toprc':toprc, 'backup':backup }
        #
        # thinf = th_start(self.top_cmd_th, th_arg)
        # self.all_th_infs.append(thinf)
        #
        # font = wx.Font(10, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        # self.label_top_cmd.SetFont(font)

        # icon
        bm = scaled_bitmap(wx.Bitmap(rtmgr_src_dir() + 'autoware_logo_2_white.png'), 0.5)
        icon = wx.EmptyIcon()
        icon.CopyFromBitmap(bm)
        self.SetIcon(icon)


    def OnRviz(self, event):
        push = event.GetEventObject()
        cmd = None

        if push.GetValue():
            self.selected_topic_dic[push] = "RViz"
            cmd = "rosrun rviz rviz -f velodyne"
            self.cmd_dic[push] = (cmd, None)
            self.launch_kill_proc2(push, self.cmd_dic)
        else:
            self.launch_kill_proc2(push, self.cmd_dic)
            val = self.selected_topic_dic.pop(push)
            print("Kill '%s'" % self.cmd_dic[push][0])
            push.SetBackgroundColour(wx.NullColour)

    def OnROSbagPlay2(self, event):
        push = event.GetEventObject()
        btn = self.button_play_rosbag_play
        tc = self.obj_to_varpanel_tc(btn, 'file')
        if tc.GetValue():
            if push == self.button_play_rosbag_play2:
                f = "self.rosbag_play_progress_bar2"
                f = eval_if_str(self, f)
                f = f if f else self.log_th
                out = subprocess.PIPE if f else None
                err = subprocess.STDOUT if f else None
                args =  ['rosbag', 'play', '--clock', tc.GetValue()]
                proc = psutil.Popen(args, stdin=subprocess.PIPE, stdout=out, stderr=err)
                self.all_procs.append(proc)
                self.proc = proc
                thinf = th_start(f, {'file':proc.stdout})
                self.push = push
                self.button_pause_rosbag_play2.Enable()
                self.button_stop_rosbag_play2.Enable()
                self.button_play_rosbag_play2.Disable()
                self.button_play_rosbag_play.Disable()
                self.button_play_rosbag_play2.SetBackgroundColour("#E0E0F0")
                self.button_stop_rosbag_play2.SetBackgroundColour(wx.NullColour)
                self.button_stop_rosbag_play2.SetForegroundColour(wx.NullColour)
                self.button_pause_rosbag_play2.SetForegroundColour(wx.NullColour)
                self.button_stop_rosbag_play2.SetValue(0)
                self.button_pause_rosbag_play2.SetValue(0)
                self.button_confirm_topics.Disable()
                self.button_confirm_depth.Disable()
            elif push == self.button_pause_rosbag_play2:
                if self.proc:
                    self.proc.stdin.write(' ')
                self.button_stop_rosbag_play2.Enable()
                self.button_pause_rosbag_play2.Enable()
            elif push == self.button_stop_rosbag_play2:
                # self.proc = self.launch_kill_proc2(self.button_play_rosbag_play2, self.cmd_dic)
                self.button_play_rosbag_play2.Enable()
                # self.button_play_rosbag_play2.SetCo
                self.button_play_rosbag_play.Enable()
                self.button_stop_rosbag_play2.Disable()
                self.button_pause_rosbag_play2.Disable()
                self.button_play_rosbag_play2.SetValue(0)
                self.button_stop_rosbag_play2.SetValue(0)
                self.button_pause_rosbag_play2.SetValue(0)
                self.button_confirm_depth.Enable()
                self.button_confirm_topics.Enable()
                dic = { (True,True):('#F9F9F8','#8B8BB9'), (True,False):('#F9F9F8','#E0E0F0') }
            	self.button_play_rosbag_play2.SetBackgroundColour(wx.NullColour)
                self.button_stop_rosbag_play2.SetBackgroundColour("#E0E0F0")
                self.button_pause_rosbag_play2.SetBackgroundColour(wx.NullColour)
                self.button_play_rosbag_play2.SetForegroundColour(wx.NullColour)
                sigint = 'SIGTERM'
                if True:
                	terminate_children(self.proc, sigint)
                terminate(self.proc, sigint)
                self.proc.wait()
                # del self.cmd_dic[self.button_play_rosbag_play2]
                if self.proc in self.all_procs:
                	self.all_procs.remove(self.proc)
        else:
            wx.MessageBox("Please Set Bag File")

    def rosbag_play_progress_bar2(self, file, ev):
    	while not ev.wait(0):
    		s = self.stdout_file_search(file, 'Duration:')
    		if not s:
    			break
    		lst = s.split()
    		pos = str_to_float(lst[0])
    		# lst[1] is '/'
    		total = str_to_float(lst[2])
    		if total == 0:
    			continue
    		prg = int(100 * pos / total + 0.5)
    		pos = str(int(pos))
    		total = str(int(total))

    		wx.CallAfter(self.label_rosbag_play_bar2.set, prg)
    		wx.CallAfter(self.label_rosbag_play_pos2.SetLabel, pos)
    		wx.CallAfter(self.label_rosbag_play_total2.SetLabel, total)
    	wx.CallAfter(self.label_rosbag_play_bar2.clear)
    	wx.CallAfter(self.label_rosbag_play_pos2.SetLabel, '')
    	wx.CallAfter(self.label_rosbag_play_total2.SetLabel, '')

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

    def OnConvertCheckedTopic(self, event):
        push = event.GetEventObject()
        cmd = None

        if push.GetValue():
            topic_output_info = self.select_created_topic[push]
            topic_var_name = topic_output_info['name']
            topic_var_panel = getattr(self, "button_" + topic_var_name)
            topic_path_obj = self.obj_to_varpanel_tc(topic_var_panel, 'file')
            topic_path = topic_path_obj.GetValue()
            topic_output_info['path'] = topic_path

            if topic_path and topic_path != 'Please Set Output Directory':
                self.select_created_topic[push] = topic_output_info
                if not os.path.exists(topic_path):
                    subprocess.call(['mkdir', '-p', topic_path])
                self.selected_topic_dic[push] = topic_output_info
                topic_type = topic_output_info['topic_type']
                if topic_type == 'sensor_msgs/Image':
                    cmd = "rosrun data_preprocessor get_Image %s %s" % (topic_path, topic_output_info['topic'])
                    # file_format = topic_output_info['topic'][1:].replace('/', '_') + "_%08d.%s"
                    # cmd = "rosrun image_view image_saver image:=%s _filename_format:=%s" % (topic_output_info['topic'], file_format)
                    self.cmd_dic[push] = (cmd, None)
                    self.launch_kill_proc2(push, self.cmd_dic)

                if topic_type == 'sensor_msgs/PointCloud2':
                    cmd = "rosrun data_preprocessor get_PCD %s %s" % (topic_path, topic_output_info['topic'])
                    self.cmd_dic[push] = (cmd, None)
                    self.launch_kill_proc2(push, self.cmd_dic)

                print("launch '%s'" % self.cmd_dic[push][0])
            else:
                push.SetValue(0)
                wx.MessageBox("Please Set Output Directory")
        else:
            self.launch_kill_proc2(push, self.cmd_dic)
            val = self.selected_topic_dic.pop(push)
            print("Kill '%s'" % self.cmd_dic[push][0])

    def OnSelectPointCheckbox(self, event):
        push = event.GetEventObject()
        if push.GetValue():
            if self.selected_pcd != {}:
                for k in self.selected_pcd.keys():
                    if k != push:
                        k.SetValue(0)
            self.pointcloud_for_depth = self.select_created_topic[push]
            self.selected_pcd[push] = self.pointcloud_for_depth
        else:
            push.SetValue(0)
            del self.selected_pcd[push]
            self.pointcloud_for_depth = None

    def OnSelectImageCheckbox(self, event):
        push = event.GetEventObject()

        if push.GetValue():
            if self.selected_img != {}:
                for k in self.selected_img.keys():
                    if k != push:
                        k.SetValue(0)
            self.image_for_depth = self.select_created_topic[push]
            self.selected_img[push] = self.image_for_depth
        else:
            push.SetValue(0)
            del self.selected_img[push]
            self.image_for_depth = None

    def OnGetConfirmTopics(self, event):
        if self.depth_flag:
            self.button_confirm_depth.SetValue(0)
            self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
            self.button_confirm_depth.SetForegroundColour(wx.NullColour)
            self.launch_kill_proc2(self.button_confirm_depth, self.cmd_dic)

        if self.objx:
            if self.objx.GetValue():
                self.objx.SetValue(0)
                self.launch_kill_proc2(self.objx, self.cmd_dic, is_rapid_delete=True)
                val = self.selected_topic_dic.pop(self.objx)
                print("Kill '%s'" % self.cmd_dic[self.objx][0])
                self.objx.SetValue(0)
                self.points_raw_save.Disable()
                self.points_raw_depth.Disable()
                self.file_url.Disable()

        if self.velodyne_button:
            if self.velodyne_button.GetValue():
                self.velodyne_button.SetValue(0)
                self.launch_kill_proc2(self.velodyne_button, self.cmd_dic, is_rapid_delete=True)
                val = self.selected_topic_dic.pop(self.velodyne_button)
                print("Kill '%s'" % self.cmd_dic[self.velodyne_button][0])
                self.velodyne_button.SetValue(0)
                self.points_raw_save.Disable()
                self.points_raw_depth.Disable()
                self.file_url.Disable()

        self.button_confirm_depth.Enable()
        self.get_confirm_topic_list()

    def get_confirm_topic_list(self):
        self.get_all_topics()
        self.get_depth_topic()

    def OnConvertVelodyne(self, event):
        push = event.GetEventObject()
        cmd = None
        dic = {
            1:"velodyne_hdl64e_s2.launch",
            2: "velodyne_hdl64e_s3.launch",
            3: "velodyne_hdl32e.launch",
            4: "velodyne_vlp16.launch",
            5: "top_urg.launch",
            6: "hokuyo_3d.launch"
        }

        if push.GetValue():
            if ((self.file_path) and (self.select)):
                self.selected_topic_dic[push] = dic[self.select]
                if not((self.select == 5) or (self.select == 6)):
                    # cmd = "roslaunch velodyne_pointcloud 32e_points.launch"
                    cmd = "roslaunch" + " data_preprocessor " + dic[self.select] + " calibration:=%s" %self.file_path
                else:
                    cmd = "roslaunch" + " data_preprocessor " + dic[self.select]
                self.cmd_dic[push] = (cmd, None)
                self.launch_kill_proc2(push, self.cmd_dic)
                if push == self.objx:
                    self.points_raw_save.Enable()
                    self.file_url.Enable()
                if push == self.velodyne_button:
                    self.points_raw_depth.Enable()
                # if push != self.velodyne_button:
                #     self.file_url.Enable()

                print("launch '%s'" % self.cmd_dic[push][0])
            else:
                push.SetValue(0)
                wx.MessageBox("Please Choose Lidar")
        else:
            self.launch_kill_proc2(push, self.cmd_dic, is_rapid_delete=True)
            val = self.selected_topic_dic.pop(push)
            print("Kill '%s'" % self.cmd_dic[push][0])
            push.SetValue(0)
            if push == self.objx:
                if self.points_raw_save.GetValue():
                    self.launch_kill_proc2(self.points_raw_save, self.cmd_dic)
                    val = self.selected_topic_dic.pop(self.points_raw_save)
                    print("Kill '%s'" % self.cmd_dic[self.points_raw_save][0])
                self.points_raw_save.SetValue(0)
                self.points_raw_save.Disable()
            self.points_raw_depth.Disable()
            # self.points_raw.Disable()
            self.file_url.Disable()

    def OnGetLidar(self, event):
        dialog = DetailDialog(self)
        try:
            dialog.ShowModal()
        finally:
            dialog.Destroy()
            if ((self.file_path) and (self.select)):
                self.objx.Enable()
            else:
                self.objx.Disable()
                # self.points_raw.Disable()
                self.points_raw_depth.Disable()
                self.points_raw_save.Disable()
                self.file_url.Disable()

    def OnGetDepthLidar(self, event):
        dialog = DetailDialog(self)
        try:
            dialog.ShowModal()
        finally:
            dialog.Destroy()
            if ((self.file_path) and (self.select)):
                self.velodyne_button.Enable()
            else:
                self.velodyne_button.Disable()
                self.points_raw_save.Disable()
                self.points_raw_depth.Disable()
                self.file_url.Disable()

    def get_bag_url(self):
        btn = self.button_play_rosbag_play
        tc = self.obj_to_varpanel_tc(btn, 'file')
        return tc.GetValue()

    def create_url_panel(self, sss, index, topic, topic_type, obj, strings, file_type="dir", comment='Please Set Output Directory'):
        button_input_name = "button_input" + str(index)
        button_input_var_name = "button_input_var" + str(index)
        dic = {
            'buttons' : {button_input_name: {'gui': {'panel': strings},'param': button_input_var_name}},
            'params' : [
                {
                    'no_save_vars' : ['file'],
                    'name': button_input_var_name,
                    'vars': [{'kind': 'path', 'name': 'file', 'path_type' : file_type, 'v': comment}]
                }]}
        self.add_params(dic.get('params', []))
        #################
        ##  set URL & param bar
        #################
        self.setup_buttons(dic.get('buttons'), self.simulation_cmd)

        button_var = getattr(self, "button_" + button_input_name)
        file_url = self.obj_to_varpanel_tc(button_var, 'file')
        if self.topic_type == "velodyne_msgs/VelodyneScan":
            self.file_url = file_url
            self.file_url.Disable()

        if file_url:
            topic_info = {}
            topic_info['path'] = file_url.GetValue()
            topic_info['topic'] = topic
            topic_info['topic_type'] = topic_type
            topic_info['name'] = button_input_name
            self.select_created_topic[obj] = topic_info

    def get_all_topic(self):
        bag_url = self.get_bag_url()
        self.select_created_topic = {}

        if bag_url:
            self.topic_and_type_list = get_type_and_topic(bag_url)
            szr = self.sizer_select_topics
            sss = self.select_scroll
            sssb = self.sizer_select_box
            if self.selected_topic_dic:
                self.delete_launch()

            self.delete_topic_panel(szr, num=0)

            topic_conversion_dic = {
                'sensor_msgs/Image' : "RGB Image",
                'sensor_msgs/PointCloud2' : "PCD",
                'velodyne_msgs/VelodyneScan' : "sensor_msgs/PointCloud2"
            }

            for i, (topic_type, topic) in enumerate(self.topic_and_type_list):
                if topic_type in topic_conversion_dic.keys():
                    select_topic_staticbox = wx.StaticBox(sss, wx.ID_ANY, "")
                    select_topic_staticbox.Lower()
                    sizer_select_topic = wx.StaticBoxSizer(select_topic_staticbox, wx.VERTICAL)
                    panelx = None
                    if topic_type == "velodyne_msgs/VelodyneScan":
                        panelx = wx.Panel(sss, wx.ID_ANY)
                        self.objx = wx.CheckBox(panelx, wx.ID_ANY, "Convert  {0}  To  {1}".format("VelodyneScan", "PointCloud2"))
                        self.objx.SetValue(0)
                        self.objx.Disable()
                        self.objx.SetForegroundColour("#FF0000")
                        self.Bind(wx.EVT_CHECKBOX, self.OnConvertVelodyne, self.objx)

                        self.buttonx = wx.ToggleButton(sss, wx.ID_ANY, _("Choose Lidar"))
                        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnGetLidar, self.buttonx)

                        upper_area = wx.BoxSizer( wx.HORIZONTAL)
                        upper_area.Add(panelx, 1, wx.ALL | wx.EXPAND, 4)
                        upper_area.Add(self.buttonx, 0, wx.ALL, 1)
                        sizer_select_topic.Add(upper_area)
                        topic = "/points_raw"

                    panel = wx.Panel(sss, wx.ID_ANY)
                    obj = wx.CheckBox(panel, wx.ID_ANY, topic)
                    obj.SetValue(0)

                    if topic_type == "velodyne_msgs/VelodyneScan":
                        self.points_raw_save = obj
                        self.topic_type = topic_type
                        topic_type = "sensor_msgs/PointCloud2"
                        self.points_raw_save.Disable()
                    else:
                        self.topic_type = None
                    obj.SetForegroundColour("#FF0000")
                    self.Bind(wx.EVT_CHECKBOX, self.OnConvertCheckedTopic, obj)

                    panel2 = wx.Panel(sss, wx.ID_ANY)
                    topic_sentence = "From  {0}  To  {1}".format(topic_type, topic_conversion_dic[topic_type])
                    obj2 = wx.StaticText(panel2, wx.ID_ANY, topic_sentence)
                    #obj2.SetForegroundColour("#FF0000")
                    font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                    obj2.SetFont(font)

                    self.panel3 = wx.Panel(sss, wx.ID_ANY)
                    self.create_url_panel(sss, i, topic, topic_type, obj, "self.panel3")

                    up_area = wx.BoxSizer( wx.HORIZONTAL)
                    down_area = wx.BoxSizer(wx.HORIZONTAL)
                    up_area.Add(panel, 1, wx.ALL | wx.EXPAND, 4)
                    up_area.Add(panel2, 3, wx.TOP | wx.EXPAND | wx.LEFT, 6)

                    sizer_select_topic.Add(up_area)
                    down_area.Add(self.panel3, 1, wx.ALL | wx.EXPAND, 4)
                    sizer_select_topic.Add(down_area, 1, wx.EXPAND, 4)
                    szr.Add(sizer_select_topic, 0, wx.ALL | wx.EXPAND, 4)
                    self.select_topic_delete_dic[0].append(sizer_select_topic)
            sss.SetSizer(szr)
            # sssb.Add(sss, 0, wx.EXPAND, 0)
            sss.Layout()
            sssb.Layout()
        else:
            wx.MessageBox("Please Set Bag File")

    def get_all_topics(self):
        bag_url = self.get_bag_url()
        self.select_created_topic = {}
        self.topic_type = None

        if bag_url:
            self.topic_and_type_list = get_type_and_topic(bag_url)
            szr = self.sizer_select_topics
            sss = self.select_scroll
            sssb = self.sizer_select_box
            if self.selected_topic_dic:
                self.delete_launch()

            self.delete_topic_panel(szr, num=0)

            topic_conversion_dic = {
                'sensor_msgs/Image' : "RGB Image",
                'sensor_msgs/PointCloud2' : "PCD",
                'velodyne_msgs/VelodyneScan' : "sensor_msgs/PointCloud2"
            }

            sizer_image_topic = None
            sizer_pointcloud_topic = None
            for topic_type, topic in self.topic_and_type_list:
                if topic_type == "sensor_msgs/Image":
                    if sizer_image_topic == None:
                        select_image_staticbox = wx.StaticBox(sss, wx.ID_ANY, "Image")
                        select_image_staticbox.Lower()
                        sizer_image_topic = wx.StaticBoxSizer(select_image_staticbox, wx.VERTICAL)

                if topic_type in ["sensor_msgs/PointCloud2", 'velodyne_msgs/VelodyneScan']:
                    if sizer_pointcloud_topic == None:
                        select_pointcloud_staticbox = wx.StaticBox(sss, wx.ID_ANY, "PointCloud")
                        select_pointcloud_staticbox.Lower()
                        sizer_pointcloud_topic = wx.StaticBoxSizer(select_pointcloud_staticbox, wx.VERTICAL)

            for i, (topic_type, topic) in enumerate(self.topic_and_type_list):
                if topic_type == "sensor_msgs/Image":
                    panelx = None
                    panel = wx.Panel(sss, wx.ID_ANY)
                    obj = wx.CheckBox(panel, wx.ID_ANY, topic)
                    obj.SetValue(0)

                    self.topic_type = None
                    obj.SetForegroundColour("#FF0000")
                    self.Bind(wx.EVT_CHECKBOX, self.OnConvertCheckedTopic, obj)

                    panel2 = wx.Panel(sss, wx.ID_ANY)
                    topic_sentence = "From  {0}  To  {1}".format(topic_type, topic_conversion_dic[topic_type])
                    obj2 = wx.StaticText(panel2, wx.ID_ANY, topic_sentence)
                    #obj2.SetForegroundColour("#FF0000")
                    font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                    obj2.SetFont(font)

                    self.panel3 = wx.Panel(sss, wx.ID_ANY)
                    self.create_url_panel(sss, i, topic, topic_type, obj, "self.panel3")

                    up_area = wx.BoxSizer( wx.HORIZONTAL)
                    down_area = wx.BoxSizer(wx.HORIZONTAL)
                    up_area.Add(panel, 1, wx.ALL | wx.EXPAND, 4)
                    up_area.Add(panel2, 3, wx.TOP | wx.EXPAND | wx.LEFT, 6)

                    sizer_image_topic.Add(up_area)
                    down_area.Add(self.panel3, 1, wx.ALL | wx.EXPAND, 4)
                    sizer_image_topic.Add(down_area, 1, wx.EXPAND, 4)

            for i, (topic_type, topic) in enumerate(self.topic_and_type_list):
                if topic_type in ["velodyne_msgs/VelodyneScan", "sensor_msgs/PointCloud2"]:
                # if topic_type in topic_conversion_dic.keys():
                    panelx = None

                    if topic_type == "velodyne_msgs/VelodyneScan":
                        panelx = wx.Panel(sss, wx.ID_ANY)
                        self.objx = wx.CheckBox(panelx, wx.ID_ANY, "Convert  {0}  To  {1}".format("VelodyneScan", "PointCloud2"))
                        self.objx.SetValue(0)
                        self.objx.Disable()
                        self.objx.SetForegroundColour("#FF0000")
                        self.Bind(wx.EVT_CHECKBOX, self.OnConvertVelodyne, self.objx)

                        self.buttonx = wx.ToggleButton(sss, wx.ID_ANY, _("Choose Lidar"))
                        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnGetLidar, self.buttonx)

                        upper_area = wx.BoxSizer( wx.HORIZONTAL)
                        upper_area.Add(panelx, 1, wx.ALL | wx.EXPAND, 4)
                        upper_area.Add(self.buttonx, 0, wx.ALL, 1)
                        sizer_pointcloud_topic.Add(upper_area)
                        topic = "/points_raw"

                    panel = wx.Panel(sss, wx.ID_ANY)
                    obj = wx.CheckBox(panel, wx.ID_ANY, topic)
                    obj.SetValue(0)

                    if topic_type == "velodyne_msgs/VelodyneScan":
                        self.points_raw_save = obj
                        self.topic_type = topic_type
                        topic_type = "sensor_msgs/PointCloud2"
                        self.points_raw_save.Disable()
                    else:
                        self.topic_type = None
                    obj.SetForegroundColour("#FF0000")
                    self.Bind(wx.EVT_CHECKBOX, self.OnConvertCheckedTopic, obj)

                    panel2 = wx.Panel(sss, wx.ID_ANY)
                    topic_sentence = "From  {0}  To  {1}".format(topic_type, topic_conversion_dic[topic_type])
                    obj2 = wx.StaticText(panel2, wx.ID_ANY, topic_sentence)
                    #obj2.SetForegroundColour("#FF0000")
                    font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                    obj2.SetFont(font)

                    self.panel3 = wx.Panel(sss, wx.ID_ANY)
                    self.create_url_panel(sss, i, topic, topic_type, obj, "self.panel3")

                    up_area = wx.BoxSizer( wx.HORIZONTAL)
                    down_area = wx.BoxSizer(wx.HORIZONTAL)
                    up_area.Add(panel, 1, wx.ALL | wx.EXPAND, 4)
                    up_area.Add(panel2, 3, wx.TOP | wx.EXPAND | wx.LEFT, 6)

                    sizer_pointcloud_topic.Add(up_area)
                    down_area.Add(self.panel3, 1, wx.ALL | wx.EXPAND, 4)
                    sizer_pointcloud_topic.Add(down_area, 1, wx.EXPAND, 4)

            if sizer_image_topic != None:
                szr.Add(sizer_image_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[0].append(sizer_image_topic)
            if sizer_pointcloud_topic != None:
                szr.Add(sizer_pointcloud_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[0].append(sizer_pointcloud_topic)
            sss.SetSizer(szr)
            sss.Layout()
            sssb.Layout()

        else:
            wx.MessageBox("Please Set Bag File")

    def delete_topic_panel(self, szr, num=0):
        topic_list = self.select_topic_delete_dic[num]

        for topic in topic_list:
            szr.Hide(topic)
            szr.Remove(topic)
        self.select_topic_delete_dic[num] = []

    def delete_launch(self):
        for k, val in self.selected_topic_dic.items():
            k.SetValue(0)
            self.launch_kill_proc2(k, self.cmd_dic)
            val = self.selected_topic_dic.pop(k)
            print("Kill '%s'" % self.cmd_dic[k][0])
            self.cmd_dic.pop(k)

    def OnConfirmDepth(self, event):
        push = event.GetEventObject()
        if self.button_confirm_depth.GetValue():
            button_var = getattr(self, "button_" + "button_input100")
            button_var2 = getattr(self, "button_" + "button_input101")
            output_url = self.obj_to_varpanel_tc(button_var, 'file').GetValue()
            calib_url = self.obj_to_varpanel_tc(button_var2, 'file').GetValue()
            if ("Please Set Output Directory" == output_url) or (not output_url) or (" " in output_url):
                self.button_confirm_depth.SetValue(0)
                self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
                self.button_confirm_depth.SetForegroundColour(wx.NullColour)
                wx.MessageBox("Please Set Correct Output Directory")
                return
            if output_url[-1] == "/":
                output_url = output_url[:-1]

            if not self.image_for_depth:
                self.button_confirm_depth.SetValue(0)
                self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
                self.button_confirm_depth.SetForegroundColour(wx.NullColour)
                wx.MessageBox("Please Select Image Topic")
                return

            if not self.pointcloud_for_depth:
                self.button_confirm_depth.SetValue(0)
                self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
                self.button_confirm_depth.SetForegroundColour(wx.NullColour)
                wx.MessageBox("Please Select Pointcloud Topic")
                return

            if ("Please Set Calibration File" == calib_url) or (not calib_url) or (" " in calib_url):
                wx.MessageBox("Please Set Correct Calibration File URL")
                self.button_confirm_depth.SetValue(0)
                self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
                self.button_confirm_depth.SetForegroundColour(wx.NullColour)
                return
            else:
                if not os.path.exists(output_url):
                    subprocess.call(['mkdir', '-p', output_url])
                cmd = "rosrun data_preprocessor get_Depth %s %s %s %s" % (output_url, calib_url, self.image_for_depth, self.pointcloud_for_depth)
                self.cmd_dic[push] = (cmd, None)
                self.launch_kill_proc2(push, self.cmd_dic)
                self.depth_flag = True
                self.button_confirm_depth.SetBackgroundColour("#8B8BB9")
                self.button_confirm_depth.SetForegroundColour("#F9F9F8")
                #):('#F9F9F8','#8B8BB9'), (True,False):('#F9F9F8','#E0E0F0')
                # self.selected_topic_dic[push] = push
                print("launch '%s'" % self.cmd_dic[push][0])
        else:
            self.launch_kill_proc2(push, self.cmd_dic)
            # val = self.selected_topic_dic.pop(push)
            self.depth_flat = False
            print("Kill '%s'" % self.cmd_dic[push][0])
            self.button_confirm_depth.SetBackgroundColour(wx.NullColour)
            self.button_confirm_depth.SetForegroundColour(wx.NullColour)

    def get_depth_topic(self):
        self.topic_type = None
        bag_url = self.get_bag_url()

        if bag_url:
            self.topic_and_type_list = get_type_and_topic(bag_url)
            szr = self.sizer_depth_topics
            sss = self.depth_scroll
            sssb = self.sizer_depth_box
            #sssb.Remove(sss)
            if self.selected_topic_dic:
                self.delete_launch()

            self.delete_topic_panel(szr, num=1)

            topic_conversion_dic = {
                'sensor_msgs/Image' : "RGB Image",
                'sensor_msgs/PointCloud2' : "PCD",
                'velodyne_msgs/VelodyneScan' : "sensor_msgs/PointCloud2"
            }

            is_image_topic = False
            is_pointcloud_topic = False
            select_image_staticbox = wx.StaticBox(sss, wx.ID_ANY, "Image")
            select_image_staticbox.Lower()
            sizer_image_topic = wx.StaticBoxSizer(select_image_staticbox, wx.VERTICAL)

            select_pointcloud_staticbox = wx.StaticBox(sss, wx.ID_ANY, "PointCloud")
            select_pointcloud_staticbox.Lower()
            sizer_pointcloud_topic = wx.StaticBoxSizer(select_pointcloud_staticbox, wx.VERTICAL)

            if True:
                select_output_staticbox = wx.StaticBox(sss, wx.ID_ANY, "Depth Output Directory")
                select_output_staticbox.Lower()
                sizer_output_topic = wx.StaticBoxSizer(select_output_staticbox, wx.VERTICAL)
                self.output_url = "output"
                self.output_depth = wx.Panel(sss, wx.ID_ANY)
                self.create_url_panel(sss, 100, "", "", self.output_url, "self.output_depth", file_type="dir", comment="Please Set Output Directory")
                output_area = wx.BoxSizer( wx.HORIZONTAL)
                output_area.Add(self.output_depth, 1, wx.ALL | wx.EXPAND, 4)
                sizer_output_topic.Add(output_area, 1, wx.EXPAND, 4)
                szr.Add(sizer_output_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_output_topic)

            if True:
                select_calib_staticbox = wx.StaticBox(sss, wx.ID_ANY, "Camera / Lidar Calibration File")
                select_calib_staticbox.Lower()
                sizer_calib_topic = wx.StaticBoxSizer(select_calib_staticbox, wx.VERTICAL)
                self.calib_url = "calib"
                self.panel_calibration = wx.Panel(sss, wx.ID_ANY)
                self.create_url_panel(sss, 101, "", "", self.calib_url, "self.panel_calibration", file_type="file", comment="Please Set Calibration File")
                calib_area = wx.BoxSizer( wx.HORIZONTAL)
                calib_area.Add(self.panel_calibration, 1, wx.ALL | wx.EXPAND, 4)
                sizer_calib_topic.Add(calib_area, 1, wx.EXPAND, 4)
                szr.Add(sizer_calib_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_calib_topic)

            for i, (topic_type, topic) in enumerate(self.topic_and_type_list):
                i = i + 30
                if topic_type == "sensor_msgs/Image":
                    is_image_topic = True
                    panel = wx.Panel(sss, wx.ID_ANY)
                    obj = wx.CheckBox(panel, wx.ID_ANY, topic)
                    obj.SetValue(0)
                    obj.SetForegroundColour("#FF0000")
                    self.Bind(wx.EVT_CHECKBOX, self.OnSelectImageCheckbox, obj)
                    self.select_created_topic[obj] = topic

                    panel2 = wx.Panel(sss, wx.ID_ANY)
                    topic_sentence = "From  {0}  To  {1}".format(topic_type, topic_conversion_dic[topic_type])
                    obj2 = wx.StaticText(panel2, wx.ID_ANY, topic_sentence)
                    font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                    obj2.SetFont(font)

                    up_area = wx.BoxSizer( wx.HORIZONTAL)
                    up_area.Add(panel, 1, wx.ALL | wx.EXPAND, 4)
                    up_area.Add(panel2, 3, wx.TOP | wx.EXPAND | wx.LEFT, 6)
                    sizer_image_topic.Add(up_area)

            for i, (topic_type, topic) in enumerate(self.topic_and_type_list):
                if topic_type == "sensor_msgs/PointCloud2" or topic_type == "velodyne_msgs/VelodyneScan":
                    is_pointcloud_topic = True
                    panelx = None
                    if topic_type == "velodyne_msgs/VelodyneScan":
                        panelx = wx.Panel(sss, wx.ID_ANY)
                        self.velodyne_button = wx.CheckBox(panelx, wx.ID_ANY, "Convert  {0}  To  {1}".format("VelodyneScan", "PointCloud2"))
                        self.velodyne_button.SetValue(0)
                        self.velodyne_button.Disable()
                        self.velodyne_button.SetForegroundColour("#FF0000")
                        self.Bind(wx.EVT_CHECKBOX, self.OnConvertVelodyne, self.velodyne_button)

                        self.buttonx = wx.ToggleButton(sss, wx.ID_ANY, _("Choose Lidar"))
                        self.Bind(wx.EVT_TOGGLEBUTTON, self.OnGetDepthLidar, self.buttonx)

                        upper_area = wx.BoxSizer( wx.HORIZONTAL)
                        upper_area.Add(panelx, 1, wx.ALL | wx.EXPAND, 4)
                        upper_area.Add(self.buttonx, 0, wx.ALL, 1)
                        sizer_pointcloud_topic.Add(upper_area)
                        topic = "/points_raw"

                    panel = wx.Panel(sss, wx.ID_ANY)
                    obj = wx.CheckBox(panel, wx.ID_ANY, topic)
                    obj.SetValue(0)

                    if topic_type == "velodyne_msgs/VelodyneScan":
                        self.points_raw_depth = obj
                        self.topic_type = topic_type
                        topic_type = "sensor_msgs/PointCloud2"
                        self.points_raw_depth.Disable()
                        self.output_depth.Enable()
                    else:
                        self.topic_type = None
                    obj.SetForegroundColour("#FF0000")
                    self.Bind(wx.EVT_CHECKBOX, self.OnSelectPointCheckbox, obj)
                    self.select_created_topic[obj] = topic

                    panel2 = wx.Panel(sss, wx.ID_ANY)
                    topic_sentence = "From  {0}  To  {1}".format(topic_type, topic_conversion_dic[topic_type])
                    obj2 = wx.StaticText(panel2, wx.ID_ANY, topic_sentence)
                    #obj2.SetForegroundColour("#FF0000")
                    font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                    obj2.SetFont(font)

                    up_area = wx.BoxSizer( wx.HORIZONTAL)
                    up_area.Add(panel, 1, wx.ALL | wx.EXPAND, 4)
                    up_area.Add(panel2, 3, wx.TOP | wx.EXPAND | wx.LEFT, 6)
                    sizer_pointcloud_topic.Add(up_area)

            if is_image_topic:
                szr.Add(sizer_image_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_image_topic)
            else:
                topic_sentence = "     Please choose the Bag File including Image Topic"
                imagepanel = wx.Panel(sss, wx.ID_ANY)
                obj = wx.StaticText(imagepanel, wx.ID_ANY, topic_sentence)
                font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                obj.SetFont(font)
                obj.SetForegroundColour("#FF0000")
                up_area = wx.BoxSizer( wx.HORIZONTAL)
                up_area.Add(imagepanel, 1, wx.ALL | wx.EXPAND, 4)
                sizer_image_topic.Add(up_area)
                szr.Add(sizer_image_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_image_topic)

            if is_pointcloud_topic:
                szr.Add(sizer_pointcloud_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_pointcloud_topic)
            else:
                topic_sentence = "     Please choose the Bag File including PointCloud Topic"
                pointcloudpanel = wx.Panel(sss, wx.ID_ANY)
                obj = wx.StaticText(pointcloudpanel, wx.ID_ANY, topic_sentence)
                font = wx.Font(13, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
                obj.SetFont(font)
                obj.SetForegroundColour("#FF0000")
                up_area = wx.BoxSizer( wx.HORIZONTAL)
                up_area.Add(pointcloudpanel, 1, wx.ALL | wx.EXPAND, 4)
                sizer_pointcloud_topic.Add(up_area)
                szr.Add(sizer_pointcloud_topic, 0, wx.ALL | wx.EXPAND, 4)
                self.select_topic_delete_dic[1].append(sizer_pointcloud_topic)
            sss.SetSizer(szr)
            sss.Layout()
            sssb.Layout()
        else:
            wx.MessageBox("Please Set Bag File")

    def OnClose(self, event):
    	# kill_all
    	for proc in self.all_procs[:]: # copy
    		(_, obj) = self.proc_to_cmd_dic_obj(proc)
    		self.launch_kill(False, 'dmy', proc, obj=obj)

    	save_dic = {}
    	for (name, pdic) in self.load_dic.items():
    		if pdic and pdic != {}:
    			prm = self.cfg_dic( {'name':name, 'pdic':pdic} ).get('param', {})
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
    		f.write(s)
    		f.close()

    	shutdown_proc_manager()

    	shutdown_sh = self.get_autoware_dir() + '/ros/shutdown'
    	if os.path.exists(shutdown_sh):
    		os.system(shutdown_sh)

    	for thinf in self.all_th_infs:
    		th_end(thinf)

    	self.Destroy()

    def ROSCb(self, data):
    	print('recv topic msg : ' + data.data)

    	r = rospy.Rate(10)
    	rospy.is_shutdown()
    	r.sleep()
    	self.pub.publish(data.data)
    	r.sleep()

    def setup_buttons(self, d, run_dic):
    	for (k,d2) in d.items():
            pfs = [ 'button_', 'checkbox_' ]
            obj = next( (self.obj_get(pf+k) for pf in pfs if self.obj_get(pf+k)), None)
            if not obj:
            	s = 'button_' + k
            	obj = StrValObj(s, False)
            	setattr(self, s, obj)
            if not d2 or type(d2) is not dict:
            	continue
            if 'run' in d2:
            	run_dic[obj] = (d2['run'], None)
            set_tooltip(obj, d2)
            gdic = self.gdic_get_1st(d2)
            if 'param' in d2:
            	pdic = self.load_dic_pdic_setup(k, d2)
            	prm = self.get_param(d2.get('param'))
            	for var in prm.get('vars'):
            		name = var.get('name')
            		if name not in pdic and 'v' in var:
            			pdic[name] = var.get('v')

            	for (name, v) in pdic.items():
            		restore = eval( gdic.get(name, {}).get('restore', 'lambda a : None') )
            		restore(v)

            	self.add_cfg_info(obj, obj, k, pdic, gdic, False, prm)

            	pnls = [ gdic.get(var.get('name'), {}).get('panel') for var in prm.get('vars') ]
            	for pnl in [ gdic.get('panel') ] + pnls:
            		if pnl:
                            self.set_param_panel(obj, eval_if_str(self, pnl))
                        #self.set_param_panel(obj, pnl)
            else:
            	self.add_cfg_info(obj, obj, k, None, gdic, False, None)

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
    	pub = rospy.Publisher('lamp_cmd', LampCmd, queue_size=10)
    	msg = LampCmd()
    	msg.l = self.button_statchk_lamp_l.GetValue()
    	msg.r = self.button_statchk_lamp_r.GetValue()
    	pub.publish(msg)

    def OnIndi(self, event):
    	pub = rospy.Publisher('indicator_cmd', IndicatorCmd, queue_size=10)
    	msg = IndicatorCmd()
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
    			set_val(b, act)

    def stat_label_off(self, obj):
    	qs_nms = [ 'map', 'sensing', 'localization', 'detection', 'mission_planning', 'motion_planning' ]
    	exec_time = self.runtime_dic.get('exec_time', {})

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
    	self.stat_dic[k] = msg.data
    	if k == 'pmap':
    		v = self.stat_dic.get(k)
    		wx.CallAfter(self.label_point_cloud.SetLabel, 'OK' if v else '')
    	if k in [ 'pmap', 'vmap' ]:
    		v = self.stat_dic.get('pmap') and self.stat_dic.get('vmap')
    		wx.CallAfter(self.label_map_qs.SetLabel, 'OK' if v else '')

    def exec_time_callback(self, msg, (key, attr)):
    	msec = int(getattr(msg, attr, 0))
    	exec_time = self.runtime_dic.get('exec_time', {})
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
    def OnTreeMotion(self, event):
    	tree = event.GetEventObject()
    	pt = event.GetPosition()
    	event.Skip()
    	(item, flags) = tree.HitTest(pt)
    	if flags & CT.TREE_HITTEST_ONITEMLABEL == 0:
    		return
    	text = item.GetData()
    	if not text:
    		return
    	x = item.GetX()
    	y = item.GetY()
    	w = item.GetWidth()
    	h = item.GetHeight()
    	(x, y) = tree.CalcScrolledPosition(x, y)
    	iw = tree.GetItemWindow(item)
    	w -= iw.GetSize()[0] if iw else 0
    	if not wx.Rect(x, y, w, h).Contains(pt):
    		return
    	(x, y) = tree.ClientToScreen((x, y))
    	self.tip_info = (tree, text, wx.Rect(x, y, w, h))
    	if getattr(self, 'tip_timer', None) is None:
    		self.tip_timer = wx.Timer(self)
    		self.Bind(wx.EVT_TIMER, self.OnTipTimer, self.tip_timer)
    	self.tip_timer.Start(200, oneShot=True)

    def OnTipTimer(self, event):
    	if getattr(self, 'tip_info', None):
    		(tree, text, rect) = self.tip_info
    		(w, h) = self.GetSize()
    		wx.TipWindow(tree, text, maxLength=w, rectBound=rect)

    def OnTreeChecked(self, event):
    	self.OnChecked_obj(event.GetItem())

    def OnChecked_obj(self, obj):
    	self.OnLaunchKill_obj(obj)

    def OnHyperlinked(self, event):
    	self.OnHyperlinked_obj(event.GetEventObject())

    def OnHyperlinked_obj(self, obj):
    	(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
    	if pdic is None or prm is None:
    		return
    	dic_list_push(gdic, 'dialog_type', 'config')
    	klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
    	dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
    	show_modal(dlg)
    	dic_list_pop(gdic, 'dialog_type')

    def obj_to_add_args(self, obj, msg_box=True):
    	(pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
    	if pdic is None or prm is None:
    		return None

    	if 'need_camera_info' in gdic.get('flags', []) and msg_box:
    		ids = self.camera_ids()
    		if ids:
    			var = self.get_var(prm, 'camera_id', {})
    			var['choices'] = ids

    			dic_list_push(gdic, 'dialog_type', 'sel_cam')
    			klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
    			dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
    			dlg_ret = show_modal(dlg)
    			dic_list_pop(gdic, 'dialog_type')
    			if dlg_ret != 0:
    				return False
    		else:
    			pdic['camera_id'] = ''

    	if 'open_dialog' in gdic.get('flags', []) and msg_box:
    		dic_list_push(gdic, 'dialog_type', 'open')
    		klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
    		dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
    		dlg_ret = show_modal(dlg)
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
    			vars.remove(var)
    			vars.append(var)

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
    		if dic_eval_if_str(self, cmd_param, 'must') and (v is None or v == ''):
    			print 'cmd_param', name, 'is required'
    			if msg_box:
    				wx.MessageBox('cmd_param ' + name + ' is required')
    			return False
    		if dic_eval_if_str(self, cmd_param, 'only_enable') and not v:
    			continue
    		if dic_eval_if_str(self, cmd_param, 'only_disable') and v:
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

    def cfg_obj_dic(self, arg_dic, sys=False, def_ret=(None,{})):
    	sys_prm = self.get_param('sys')
    	prm_chk = {
    		True  : (lambda prm : prm is sys_prm),
    		False : (lambda prm : prm is not sys_prm),
    		None  : (lambda prm : True) }.get(sys)
    	arg_dic_chk = lambda dic: all( [ dic.get(k) == v for (k,v) in arg_dic.items() ] )
    	return next( ( (cfg_obj, dic) for (cfg_obj, dic) in self.config_dic.items() \
    		       if arg_dic_chk(dic) and prm_chk(dic.get('param')) ), def_ret)

    def cfg_dic(self, arg_dic, sys=False, def_ret={}):
    	(_, dic) = self.cfg_obj_dic(arg_dic, sys=sys, def_ret=(None, def_ret))
    	return dic

    def cfg_prm_to_obj(self, arg_dic, sys=False):
    	return self.cfg_dic(arg_dic, sys=sys).get('obj')

    def name_to_pdic_gdic_prm(self, name, sys=False):
    	d = self.cfg_dic( {'name':name}, sys=sys )
    	return ( d.get('pdic'), d.get('gdic'), d.get('param') )

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
    		hook_var = gdic_v.get('hook_var', {})
    		every_time = 'every_time' in hook_var.get('flags', [])
    		if var == gdic.get('update_func_arg_var') or every_time:
    			hook = hook_var.get('hook')
    			if hook:
    				hook(hook_var.get('args', {}))

    	if 'pub' in prm:
    		self.publish_param_topic(pdic, prm)
    	self.rosparam_set(pdic, prm)
    	self.update_depend_enable(pdic, gdic, prm)

    	d = self.cfg_dic( {'pdic':pdic, 'gdic':gdic, 'param':prm}, sys=True )
    	self.update_proc_cpu(d.get('obj'), d.get('pdic'), d.get('param'))

    def update_proc_cpu(self, obj, pdic=None, prm=None):
    	if obj is None or not obj.GetValue():
    		return
    	(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
    	if proc is None:
    		return
    	if pdic is None or prm is None:
    		(pdic, _, prm) = self.obj_to_pdic_gdic_prm(obj, sys=True)

    	cpu_chks = self.param_value_get(pdic, prm, 'cpu_chks')
    	cpu_chks = cpu_chks if cpu_chks else [ True for i in range(get_cpu_count()) ]
    	cpus = [ i for i in range(get_cpu_count()) if cpu_chks[i] ]
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
    		enables_set(vp, 'depend', v)

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
    	rosparams = None
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
    		if rosparams is None:
    			cmd = [ 'rosparam', 'list' ]
    			rosparams = subprocess.check_output(cmd).strip().split('\n')
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
                print("ROSparam_set")
                print(cmd)
                subprocess.call(cmd)

    #
    # Sensing Tab
    #
    def OnSensingDriver(self, event):
    	self.OnChecked_obj(event.GetEventObject())

    def OnROSbagRecord(self, event):
    	self.dlg_rosbag_record.Show()
    	obj = event.GetEventObject()
    	set_val(obj, False)

    def create_checkboxes(self, dic, panel, sizer, probe_dic, run_dic, bind_handler):
    	if 'name' not in dic:
    		return
    	obj = None
    	bdr_flg = wx.ALL
    	if 'subs' in dic:
    		lst = []
    		for d in dic['subs']:
    			self.create_checkboxes(d, panel, lst, probe_dic, run_dic, bind_handler)
    		if dic['name']:
    			obj = static_box_sizer(panel, dic.get('name'))
    			set_tooltip(obj.GetStaticBox(), dic)
    		else:
    			obj = wx.BoxSizer(wx.VERTICAL)
    		for (o, flg) in lst:
    			obj.Add(o, 0, wx.EXPAND | flg, 4)
    	else:
    		obj = wx.CheckBox(panel, wx.ID_ANY, dic['name'])
    		set_tooltip(obj, dic)
    		self.Bind(wx.EVT_CHECKBOX, bind_handler, obj)
    		bdr_flg = wx.LEFT | wx.RIGHT
    		if 'probe' in dic:
    			probe_dic[obj] = (dic['probe'], None)
    		if 'run' in dic:
    			run_dic[obj] = (dic['run'], None)
    		if 'param' in dic:
    			obj = self.add_config_link(dic, panel, obj)
    		else:
    			gdic = self.gdic_get_1st(dic)
    			self.add_cfg_info(obj, obj, dic.get('name'), None, gdic, False, None)
    	if sizer is not None:
    		sizer.append((obj, bdr_flg))
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

    def camera_ids(self):
    	if self.button_synchronization.GetValue():
    		return []
    	cmd = "rostopic list | sed -n 's|/image_raw||p' | sed s/^$//"
    	return subprocess.check_output(cmd, shell=True).strip().split()

    def cam_id_to_obj(self, cam_id, v):
    	cam_id_obj = self.cfg_prm_to_obj( {'name':cam_id} )
    	if cam_id_obj is None:
    		cam_id_obj = StrValObj(cam_id, v)
    	cam_id_obj.SetValue(v)
    	return cam_id_obj

    def camera_id_hook(self, args):
    	new_id = args.get('pdic', {}).get('camera_id', '')
    	ids = args.get('ids', [])
    	if new_id not in ids:
    		return
    	idx = ids.index(new_id)

    	pp = args.get('param_panel')
    	if pp:
    		pp.detach_func()
    	dlg = args.get('dlg')
    	if dlg:
    		dlg.EndModal(idx + 100)

    def OnCalibrationPublisher(self, event):
    	obj = event.GetEventObject()
    	(_, gdic_org, prm) = self.obj_to_pdic_gdic_prm(obj)
    	if obj.GetValue():
    		gdic_org['ids'] = self.camera_ids()
    	ids = gdic_org.get('ids', [])

    	if ids == []:
    		self.OnLaunchKill(event)
    		return
    	#
    	# setup
    	#
    	(cmd_dic, cmd, _) = self.obj_to_cmd_dic_cmd_proc(obj)

    	flags = gdic_org.get('flags', [])[:] # copy
    	if 'open_dialog' in flags:
    		flags.remove('open_dialog')

    	pdic_baks = {}
    	for cam_id in ids:
    		(pdic_a, gdic_a, _) = self.name_to_pdic_gdic_prm(cam_id)
    		pdic = pdic_a if pdic_a else self.load_dic_pdic_setup(cam_id, {})
    		pdic_baks[cam_id] = pdic.copy()
    		gdic = gdic_a if gdic_a else gdic_org.copy()
    		gdic['flags'] = flags

    		cam_id_obj = self.cam_id_to_obj(cam_id, obj.GetValue())
    		if not pdic_a or not gdic_a:
    			self.add_cfg_info(cam_id_obj, cam_id_obj, cam_id, pdic, gdic, False, prm)
    		if not cam_id_obj in cmd_dic:
    			cmd_dic[ cam_id_obj ] = (cmd, None)

    	var = self.get_var(prm, 'camera_id', {})
    	var['choices'] = ids

    	#
    	# Dialog
    	#
    	cam_id = ids[0]
    	while obj.GetValue():
    		(pdic, gdic, _) = self.name_to_pdic_gdic_prm(cam_id)
    		pdic['camera_id'] = cam_id
    		dic_list_push(gdic, 'dialog_type', 'open2')
    		klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
    		dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)

    		gdic_v = dic_getset(gdic, 'camera_id', {})
    		args = { 'pdic':pdic, 'ids':ids, 'param_panel':gdic.get('param_panel'), 'dlg':dlg }
    		gdic_v['hook_var'] = { 'hook':self.camera_id_hook, 'args':args }

    		dlg_ret = show_modal(dlg)

    		dic_list_pop(gdic, 'dialog_type')
    		pdic['camera_id'] = cam_id # restore

    		if dlg_ret == 0: # OK
    			break

    		idx = dlg_ret - 100
    		if idx < 0 or len(ids) <= idx: # Cancel
    			for cam_id in ids:
    				(pdic, _, _) = self.name_to_pdic_gdic_prm(cam_id)
    				pdic.update(pdic_baks.get(cam_id))
    			set_val(obj, False)
    			return

    		# Menu changed
    		cam_id = ids[idx]

    	#
    	# Launch / Kill
    	#
    	for cam_id in ids:
    		cam_id_obj = self.cfg_prm_to_obj( {'name':cam_id} )
    		(pdic, _, _) = self.obj_to_pdic_gdic_prm(cam_id_obj)
    		pdic['solo_camera'] = False
    		#print '@', cam_id, cam_id_obj.GetValue()
    		self.OnLaunchKill_obj(cam_id_obj)

    #
    # Input ROSbag File Tab
    #
    def rosbag_info_hook(self, v):
    	if not v:
    		return
    	th_start(self.rosbag_info_hook_th, {'v':v} )

    def rosbag_info_hook_th(self, ev, v):  # thread
    	err = subprocess.STDOUT
    	s = subprocess.check_output([ 'rosbag', 'info', v ], stderr=err).strip()
    	wx.CallAfter(self.label_rosbag_info.SetLabel, s)
    	wx.CallAfter(self.label_rosbag_info.GetParent().FitInside)


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
    	cpu_n = get_cpu_count()

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
    			fv = str_to_float(v)
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
    			col = self.info_col(str_to_float(load), rate_per_cpu_yellow, rate_per_cpu, (64,64,64), (200,0,0))
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

    def OnLaunchKill(self, event):
    	self.OnLaunchKill_obj(event.GetEventObject())

    def OnLaunchKill_obj(self, obj):
    	self.alias_sync(obj)
    	obj = self.alias_grp_top_obj(obj)
    	v = obj.GetValue()
    	add_args = self.obj_to_add_args(obj, msg_box=v) # no open dialog at kill
        # print("add_args", add_args)
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

    def OnROSbagPlay(self, event):
    	obj = event.GetEventObject()

    	play = self.button_play_rosbag_play
    	stop = self.button_stop_rosbag_play
    	pause = self.button_pause_rosbag_play

    	(_, _, prm) = self.obj_to_pdic_gdic_prm(play)
    	var = self.get_var(prm, 'sim_time', {})

    	if obj == play:
    		var['v'] = True
    		self.OnLaunchKill_obj(play)
    		button_color_change(play); self.button_confirm_depth.Disable()
    		set_val(stop, False);self.button_play_rosbag_play2.Disable()
    		set_val(pause, False);self.button_confirm_topics.Disable()
    	elif obj == stop:
    		set_val(stop, True); self.button_confirm_depth.Enable()
    		set_val(play, False); self.button_play_rosbag_play2.Enable()
    		set_val(pause, False);self.button_confirm_topics.Enable()
    		var['v'] = False
    		self.OnLaunchKill_obj(play)
    		button_color_change(stop)
    	elif obj == pause:
    		(_, _, proc) = self.obj_to_cmd_dic_cmd_proc(play)
    		if proc:
    			proc.stdin.write(' ')

    def OnFtrace(self, event):
    	obj = event.GetEventObject()
    	cmd = 'rosrun runtime_manager ftrace.py'
    	v = obj.GetValue()
    	self.ftrace_proc_ = self.launch_kill(v, cmd,
    		None if v else self.ftrace_proc_, obj=obj)

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
    		wx.CallAfter(self.label_point_cloud_bar.set, 100 * i / n)
    	wx.CallAfter(self.label_point_cloud_bar.clear)

    # thread
    def rosbag_play_progress_bar(self, file, ev):
    	while not ev.wait(0):
    		s = self.stdout_file_search(file, 'Duration:')
    		if not s:
    			break
    		lst = s.split()
    		pos = str_to_float(lst[0])
    		# lst[1] is '/'
    		total = str_to_float(lst[2])
    		if total == 0:
    			continue
    		prg = int(100 * pos / total + 0.5)
    		pos = str(int(pos))
    		total = str(int(total))

    		wx.CallAfter(self.label_rosbag_play_bar.set, prg)
    		wx.CallAfter(self.label_rosbag_play_pos.SetLabel, pos)
    		wx.CallAfter(self.label_rosbag_play_total.SetLabel, total)
    	wx.CallAfter(self.label_rosbag_play_bar.clear)
    	wx.CallAfter(self.label_rosbag_play_pos.SetLabel, '')
    	wx.CallAfter(self.label_rosbag_play_total.SetLabel, '')

    def alias_sync(self, obj, v=None):
    	en = None
    	if getattr(obj, 'IsEnabled', None):
    		(key, en) = enables_get_last(obj)
    		if not key:
    			en = obj.IsEnabled()
    	grp = self.alias_grp_get(obj)
    	if getattr(obj, 'GetValue', None):
    		v = obj.GetValue()
    	for o in grp:
    		if o is obj:
    			continue

    		if en is not None and o.IsEnabled() != en and not self.is_toggle_button(o):
    			if key:
    				enable_set(o, key, en)
    			else:
    				o.Enable(en)
    		if v is not None and getattr(o, 'SetValue', None):
    			set_val(o, v)
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
    		tree.Bind(wx.EVT_MOTION, self.OnTreeMotion)
    	else:
    		ct_type = 1 if 'cmd' in items else 0 # 1:checkbox type
    		item = tree.AppendItem(item, name, ct_type=ct_type)
    		if 'desc' in items:
    			item.SetData(items.get('desc'))
    		if 'cmd' in items:
    			cmd_dic[item] = (items['cmd'], None)

    			pdic = self.load_dic_pdic_setup(name, items)
    			pnl = wx.Panel(tree, wx.ID_ANY)
    			add_objs = []
    			self.new_link(item, name, pdic, self.sys_gdic, pnl, 'sys', 'sys', add_objs)
    			gdic = self.gdic_get_1st(items)
    			if 'param' in items:
    				self.new_link(item, name, pdic, gdic, pnl, 'app', items.get('param'), add_objs)
    			else:
    				self.add_cfg_info(item, item, name, None, gdic, False, None)
    			szr = sizer_wrap(add_objs, wx.HORIZONTAL, parent=pnl)
    			szr.Fit(pnl)
    			tree.SetItemWindow(item, pnl)

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

    	(cmd, proc) = cmd_dic[obj]
    	if not cmd:
    		set_val(obj, False)

    	proc = self.launch_kill(v, cmd, proc, add_args, obj=obj, kill_children=True)

    	(cfg_obj, dic) = self.cfg_obj_dic( {'obj':obj} )
    	if cfg_obj and dic.get('run_disable'):
    		cfg_obj.Enable(not v)

    	cmd_dic[obj] = (cmd, proc)
    	if not v:
    		self.stat_label_off(obj)

    def launch_kill_proc2(self, obj, cmd_dic, add_args=None, is_rapid_delete=None):
    	v = obj.GetValue()

    	(cmd, proc) = cmd_dic[obj]
    	if not cmd:
    		set_val(obj, False)

    	proc = self.launch_kill(v, cmd, proc, add_args, obj=obj, is_rapid_delete=is_rapid_delete)

    	self.cmd_dic[obj] = (cmd, proc); return proc

    def proc_to_cmd_dic_obj(self, proc):
    	for cmd_dic in self.all_cmd_dics:
    		obj = next( (obj for (obj, v) in cmd_dic.items() if proc in v), None)
    		if obj:
    			return (cmd_dic, obj)
    	return (None, None)

    def launch_kill(self, v, cmd, proc, add_args=None, sigint=None, obj=None, kill_children=None, is_rapid_delete=False):
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

    		f = self.obj_to_gdic(obj, {}).get('stdout_func')
    		f = eval_if_str(self, f)
    		f = f if f else self.log_th

    		out = subprocess.PIPE if f else None
    		err = subprocess.STDOUT if f else None
    		if f == self.log_th:
    			err = subprocess.PIPE

    		proc = psutil.Popen(args, stdin=subprocess.PIPE, stdout=out, stderr=err)
    		self.all_procs.append(proc)

    		if f == self.log_th:
    			thinf = th_start(f, {'file':proc.stdout, 'que':self.log_que_stdout})
    			self.all_th_infs.append(thinf)
    			thinf = th_start(f, {'file':proc.stderr, 'que':self.log_que_stderr})
    			self.all_th_infs.append(thinf)
    		elif f:
    			thinf = th_start(f, {'file':proc.stdout})
    			self.all_th_infs.append(thinf)
    	else:
    		flags = self.obj_to_gdic(obj, {}).get('flags', [])
    		if sigint is None:
    			sigint = 'SIGTERM' not in flags
                if is_rapid_delete:
                    sigint = False
                    flags = ["SIGTERM", "kill_children"]
    		if kill_children is None:
    			kill_children = 'kill_children' in flags
    		if kill_children:
    			terminate_children(proc, sigint)
    		terminate(proc, sigint)
    		proc.wait()
    		if proc in self.all_procs:
    			self.all_procs.remove(proc)
    		proc = None

    	return proc

    def roslaunch_to_nodes(self, cmd):
    	try:
    		s = subprocess.check_output(cmd).strip()
    		return s.split('\n') if s != '' else []
    	except subprocess.CalledProcessError:
    		return []

    def set_bg_all_tabs(self, col=wx.NullColour):

    	add_pnls = [
    		self,
    		# self.tree_ctrl_0,
    		# self.tree_ctrl_1,
    		# self.tree_ctrl_data
        ]

    	for tab in self.all_tabs + add_pnls:
    		tab.SetBackgroundColour(col)

    def get_autoware_dir(self):
    	dir = rtmgr_src_dir() + '../../../../../../'
    	return os.path.abspath(dir)

    def load_yaml(self, filename, def_ret=None):
    	return load_yaml(filename, def_ret)

    def toggle_enable_obj(self, obj):
    	objs = []
    	pfs = [ 'button_play_', 'button_stop_', 'button_pause_',
    		'button_ref_', 'text_ctrl_' ]
    	key = self.obj_key_get(obj, pfs)
    	if key:
    		objs += self.key_objs_get(pfs, key)

    	gdic = self.obj_to_gdic(obj, {})
    	objs += [ eval_if_str(self, e) for e in gdic.get('ext_toggle_enables', []) ]

    	self.toggle_enables(objs)

    def toggle_enables(self, objs):
    	for obj in objs:
    		if getattr(obj, 'IsEnabled', None):
    			en = enables_get(obj, 'toggle', obj.IsEnabled())
    			enables_set(obj, 'toggle', not en)
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

    def name_get_cond(self, obj, cond=(lambda s : True), def_ret=None):
    	return next( (nm for nm in dir(self) if cond(nm) and getattr(self, nm) is obj), def_ret)

    def val_get(self, name):
    	obj = self.obj_get(name)
    	if obj is None:
    		return None
    	return obj.GetValue() if getattr(obj, 'GetValue', None) else None

    def obj_get(self, name):
    	return getattr(self, name, None)

def gdic_dialog_type_chk(gdic, name):
	dlg_type = dic_list_get(gdic, 'dialog_type', 'config')

	tail = '_dialog_only'
	lst = [ (k, k[:-len(tail)]) for k in gdic.keys() if k[-len(tail):] == tail ]
	only_chk = next( (False for (k,type) in lst if type != dlg_type and name in gdic.get(k, [])), True)

	tail = '_dialog_allow'
	lst = [ (k, k[:-len(tail)]) for k in gdic.keys() if k[-len(tail):] == tail ]
	allow_chk = next( (False for (k,type) in lst if type == dlg_type and name not in gdic.get(k, [])), True)

	return only_chk and allow_chk

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

		self.gdic['param_panel'] = self

		obj = self.frame.cfg_prm_to_obj( {'pdic':self.pdic, 'gdic':self.gdic, 'param':self.prm} )
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

			gdic_v = self.get_gdic_v_and_chk_enable(name)
			if gdic_v is None:
				continue

			bak_stk_push(gdic_v, 'func')
			if gdic_v.get('func'):
				continue

			v = self.pdic.get(name, var.get('v'))

			vp = VarPanel(self, var=var, v=v, update=self.update)
			vp.setup_tooltip()
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
				enables_set(vp, 'toggle', proc is None)

			if 'disable' in gdic_v.get('flags', []):
				vp.Enable(False)
			if 'hide' in gdic_v.get('flags', []):
				vp.Hide()

		self.SetSizer(szr)
		if 'no_init_update' not in self.prm.get('flags', []):
			self.update()

	def get_gdic_v_and_chk_enable(self, var_name):
                gdic_v = dic_getset(self.gdic, var_name, {})
		if 'panel' in gdic_v and dic_eval_if_str(self.frame, gdic_v, 'panel') != self.GetParent():
			return None
		return gdic_v

	def update(self, var=None):
		update_func = self.gdic.get('update_func')
		if update_func:
			self.gdic['update_func_arg_var'] = var
			update_func(self.pdic, self.gdic, self.prm)

	def detach_func(self):
		for var in self.prm.get('vars'):
			name = var.get('name')

			if not gdic_dialog_type_chk(self.gdic, name):
				continue

			gdic_v = self.get_gdic_v_and_chk_enable(name)
			if gdic_v is None:
				continue

			if 'func' in gdic_v:
				bak_stk_pop(gdic_v, 'func')

			vp = gdic_v.get('var')
			lst_remove_once(self.gdic.get('ext_toggle_enables', []), vp)

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
		self.lb = None

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
				self.lb = wx.StaticText(self, wx.ID_ANY, label)
				flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
				sizer_wrap((self.lb, self.obj), wx.HORIZONTAL, 0, flag, 4, self)
			return
		if self.kind == 'checkbox':
			self.obj = wx.CheckBox(self, wx.ID_ANY, label)
			self.obj.SetValue(v)
			self.Bind(wx.EVT_CHECKBOX, self.OnUpdate, self.obj)
			return
		if self.kind == 'checkboxes':
			item_n = dic_eval_if_str(self, self.var, 'item_n', 1)
			self.obj = Checkboxes(self, item_n, label)
			self.obj.set(v)
			for box in self.obj.boxes:
				self.obj.Bind(wx.EVT_CHECKBOX, self.OnUpdate, box)
			return
		if self.kind == 'toggle_button':
			self.obj = wx.ToggleButton(self, wx.ID_ANY, label)
			set_val(self.obj, v)
			self.Bind(wx.EVT_TOGGLEBUTTON, self.OnUpdate, self.obj)
			button_color_hdr_setup(self.obj)
			return
		if self.kind == 'hide':
			self.Hide()
			return

		szr = wx.BoxSizer(wx.HORIZONTAL)

		self.lb = wx.StaticText(self, wx.ID_ANY, label)
		flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
		szr.Add(self.lb, 0, flag, 4)

		if self.kind == 'path':
			v = str(v)
			v = path_expand_cmd(v)
			v = os.path.expandvars(os.path.expanduser(v))

		style = wx.TE_PROCESS_ENTER + wx_flag_get( self.var.get('str_flags', []) )

		self.tc = wx.TextCtrl(self, wx.ID_ANY, str(v), style=style)
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
			button_color_hdr_setup(self.ref)
			self.ref.SetMinSize((40,29))
			szr.Add(self.ref, 0, flag, 4)

		if self.has_slider or self.kind == 'num':
			vszr = wx.BoxSizer(wx.VERTICAL)
			vszr.Add( self.create_bmbtn("inc.png", self.OnIncBtn) )
			vszr.Add( self.create_bmbtn("dec.png", self.OnDecBtn) )
			szr.Add(vszr, 0, wx.ALIGN_CENTER_VERTICAL)

		self.SetSizer(szr)

	def setup_tooltip(self):
		if get_tooltips(self.var):
			set_tooltips(self.obj, self.var)
		if get_tooltip(self.var):
			obj = self.lb if self.lb else (self if self.kind == 'radio_box' else self.obj)
			set_tooltip(obj, self.var)

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
		v = str_to_float(s) if self.is_float else int(s)
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
		self.update(self.var)

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
			self.update(self.var)

	def OnUpdate(self, event):
		if self.has_slider:
			self.slider.SetValue(self.get_int_v())
		self.update(self.var)

	def OnRef(self, event):
		if file_dialog(self, self.tc, self.var) == wx.ID_OK:
			self.update(self.var)

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

class MyDialogDPM(rtmgr.MyDialogDPM):
	def __init__(self, *args, **kwds):
		pdic = kwds.pop('pdic')
		self.pdic_bak = pdic.copy()
		gdic = kwds.pop('gdic')
		prm = kwds.pop('prm')
		rtmgr.MyDialogDPM.__init__(self, *args, **kwds)

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
		dic = { self.hyperlink_car : self.frame.button_car_dpm,
			self.hyperlink_pedestrian : self.frame.button_pedestrian_dpm }
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
		obj = getattr(self.frame, 'button_' + obj_key, None) if obj_key else None
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

		name = 'lane_stop'
		var = next( ( var for var in self.prm.get('vars', []) if var.get('name') == name ), {} )
		v = self.pdic.get( name, var.get('v', False) )
		set_val(self.checkbox_lane_stop, v)

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
		v = event.GetEventObject().GetValue()
		self.pdic['lane_stop'] = v
		msg.manual_detection = not v
		pub.publish(msg)

	def OnOk(self, event):
		self.EndModal(0)

	def OnCancel(self, event):
		self.EndModal(-1)

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
		(col1, col2) = (wx.Colour(0,0,250), wx.Colour(0,0,128))
		if prg >= self.lmt_bar_prg:
			(col1, col2) = (wx.Colour(250,0,0), wx.Colour(128,0,0))
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

		self.dflt_col1 = wx.Colour(250,250,250)
		self.dflt_col2 = wx.Colour(128,128,128)
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

class StrValObj:
	def __init__(self, s, v):
		self.s = s
		self.v = v
	def GetValue(self):
		return self.v
	def SetValue(self, v):
		self.v = v

class MyApp(wx.App):
	def OnInit(self):
		wx.InitAllImageHandlers()
		frame_1 = Final(None, wx.ID_ANY, "")
		self.SetTopWindow(frame_1)
		buttons_color_hdr_setup(frame_1)
		frame_1.Show()
		return 1

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
	ret = show_modal(dlg)
	if ret == wx.ID_OK:
		path = ','.join(dlg.GetPaths()) if path_type == 'multi' else dlg.GetPath()
		if path_type == 'dir' and fns:
			path = ','.join([ path + '/' + fn for fn in fns ])
		set_path(tc, path)
	dlg.Destroy()
	return ret

def button_color_change(btn, v=None):
	if v is None and type(btn) is wx.ToggleButton:
		v = btn.GetValue()
	key = ( v , btn.IsEnabled() )
	dic = { (True,True):('#F9F9F8','#8B8BB9'), (True,False):('#F9F9F8','#E0E0F0') }
	(fcol, bcol) = dic.get(key, (wx.NullColour, wx.NullColour))
	btn.SetForegroundColour(fcol)
	btn.SetBackgroundColour(bcol)

def OnButtonColorHdr(event):
	btn = event.GetEventObject()
	dic = { wx.EVT_TOGGLEBUTTON.typeId : None,
		wx.EVT_LEFT_DOWN.typeId	   : True,
		wx.EVT_LEFT_UP.typeId	   : False }
	v = dic.get(event.GetEventType(), '?')
	if v != '?':
		button_color_change(btn, v)
	event.Skip()

btn_null_bgcol = None

def is_btn_null_bgcol(btn):
	global btn_null_bgcol
	bak = btn.GetBackgroundColour()
	if btn_null_bgcol is None:
		btn.SetBackgroundColour(wx.NullColour)
		btn_null_bgcol = btn.GetBackgroundColour()
		if bak != btn_null_bgcol:
			btn.SetBackgroundColour(bak)
	return bak == btn_null_bgcol

def button_color_hdr_setup(btn):
	hdr = OnButtonColorHdr
	if type(btn) is wx.ToggleButton:
		btn.Bind(wx.EVT_TOGGLEBUTTON, hdr)
	elif type(btn) is wx.Button and is_btn_null_bgcol(btn):
		btn.Bind(wx.EVT_LEFT_DOWN, hdr)
		btn.Bind(wx.EVT_LEFT_UP, hdr)

def buttons_color_hdr_setup(frm_obj):
	key = 'button_'
	btns = [ getattr(frm_obj, nm) for nm in dir(frm_obj) if nm[:len(key)] == key ]
	for btn in btns:
		button_color_hdr_setup(btn)

def show_modal(dlg):
	buttons_color_hdr_setup(dlg)
	return dlg.ShowModal()

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

def get_tooltip(dic):
	return dic.get('desc')

def get_tooltips(dic):
	return dic.get('descs', [])

def set_tooltip(obj, dic):
	set_tooltip_str(obj, get_tooltip(dic))

def set_tooltip_str(obj, s):
	if s and getattr(obj, 'SetToolTipString', None):
		obj.SetToolTipString(s)

def set_tooltips(obj, dic):
	lst = get_tooltips(dic)
	if lst and getattr(obj, 'SetItemToolTip', None):
		for (ix, s) in enumerate(lst):
			obj.SetItemToolTip(ix, s)

def get_tooltip_obj(obj):
	if getattr(obj, 'GetToolTip', None):
		t = obj.GetToolTip()
		return t.GetTip() if t else None
	return None

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
		'center_v' : wx.ALIGN_CENTER_VERTICAL, 'center_h' : wx.ALIGN_CENTER_HORIZONTAL,
		'passwd' : wx.TE_PASSWORD }
	lst = [ dic.get(f) for f in flags if f in dic ]
	return reduce(lambda a,b : a+b, [0] + lst)

def msg_path_to_obj_attr(msg, path):
	lst = path.split('.')
	obj = msg
	for attr in lst[:-1]:
		obj = getattr(obj, attr, None)
	return (obj, lst[-1])

def str_to_rosval(s, type_str, def_ret=None):
	cvt_dic = {
		'int8':int , 'int16':int , 'int32':int ,
		'uint8':int , 'uint16':int , 'uint32':int ,
		'int64':long , 'uint64':long,
		'float32':float, 'float64':float,
	}
	t = cvt_dic.get(type_str)
	s = s.replace(',','.') if t is float and type(s) is str else s
	return t(s) if t else def_ret

def str_to_float(s):
	return float( s.replace(',','.') )

def set_path(tc, v):
	tc.SetValue(v)
	tc.SetInsertionPointEnd()

def set_val(obj, v):
	func = getattr(obj, 'SetValue', getattr(obj, 'Check', None))
	if func:
		func(v)
		obj_refresh(obj)
	if type(obj) is wx.ToggleButton:
		button_color_change(obj)

def enables_set(obj, k, en):
	d = attr_getset(obj, 'enabLes', {})
	d[k] = en
	d['last_key'] = k
	obj.Enable( all( d.values() ) )
	if isinstance(obj, wx.HyperlinkCtrl):
		if not hasattr(obj, 'coLor'):
			obj.coLor = { True:obj.GetNormalColour(), False:'#808080' }
		c = obj.coLor.get(obj.IsEnabled())
		obj.SetNormalColour(c)
		obj.SetVisitedColour(c)

def enables_get(obj, k, def_ret=None):
	return attr_getset(obj, 'enabLes', {}).get(k, def_ret)

def enables_get_last(obj):
	k = enables_get(obj, 'last_key')
	return (k, enables_get(obj, k))

def obj_refresh(obj):
	if type(obj) is CT.GenericTreeItem:
		while obj.GetParent():
			obj = obj.GetParent()
		tree = obj.GetData()
		tree.Refresh()

# dic_list util (push, pop, get)
def dic_list_push(dic, key, v):
	dic_getset(dic, key, []).append(v)

def dic_list_pop(dic, key):
	dic.get(key, [None]).pop()

def dic_list_get(dic, key, def_ret=None):
	return dic.get(key, [def_ret])[-1]

def bak_stk_push(dic, key):
	if key in dic:
		k = key + '_bak_str'
		dic_getset(dic, k, []).append( dic.get(key) )

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


def attr_getset(obj, name, def_ret):
	if not hasattr(obj, name):
		setattr(obj, name, def_ret)
	return getattr(obj, name)

def dic_getset(dic, key, def_ret):
	if key not in dic:
		dic[key] = def_ret
	return dic.get(key)

def lst_append_once(lst, v):
	exist = v in lst
	if not exist:
		lst.append(v)
	return exist

def lst_remove_once(lst, v):
	exist = v in lst
	if exist:
		lst.remove(v)
	return exist

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

def eval_if_str(self, v):
	return eval(v) if type(v) is str else v

def dic_eval_if_str(self, dic, key, def_ret=None):
	return eval_if_str( self, dic.get(key, def_ret) )

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

def get_cpu_count():
	try:
		return psutil.NUM_CPUS
	except AttributeError:
		return psutil.cpu_count()


class DetailDialog(wx.Dialog):
    def __init__(self, parent):
        wx.Dialog.__init__(self, parent, title="modal window", size=(370, 260))

        panel = wx.Panel(self, wx.ID_ANY)
        layout = wx.BoxSizer(wx.VERTICAL)

        self.filename = os.getcwd() + "/src/util/packages/data_preprocessor/scripts/32db.yaml"
        self.tc2 = wx.TextCtrl(panel, wx.ID_ANY, self.filename, style=1024)
        self.tc2.SetMinSize((300, 29))
        self.Bind(wx.EVT_TEXT_ENTER, self.update_path, self.tc2)

        ref = wx.Button(panel, wx.ID_ANY, 'Ref')
    	self.Bind(wx.EVT_BUTTON, self.open_dialog, ref)
    	ref.SetMinSize((40,29))
        self.dir = ""

        sb = wx.StaticBox(panel, wx.ID_ANY, "Spam, spam, spam")
        sb.SetLabel("Lidar's Calibration File")
        layout1 = wx.StaticBoxSizer(sb, wx.HORIZONTAL)
        layout1.Add(self.tc2, 0, wx.ALL, 4)
    	layout1.Add(ref, 0, wx.ALL, 4)
        layout.Add(layout1)

        button = wx.Button(panel, wx.ID_ANY, "OK")
        button2 = wx.Button(panel, wx.ID_ANY, "Cancel")
        ch1 = wx.CheckBox(panel, wx.ID_ANY, "Velodyne HDL-64e-S2")
        ch2 = wx.CheckBox(panel, wx.ID_ANY, "Velodyne HDL-64e-S3")
        ch3 = wx.CheckBox(panel, wx.ID_ANY, "Velodyne HDL-32e")
        ch4 = wx.CheckBox(panel, wx.ID_ANY, "Velodyne VLP-16")
        ch5 = wx.CheckBox(panel, wx.ID_ANY, "Hokuyo TOP-URG")
        ch6 = wx.CheckBox(panel, wx.ID_ANY, "Hokuyo 3D-URG")
        self.select = ""
        self.parent = parent
        self.values = {ch1:1, ch2:2, ch3:3, ch4:4, ch5:5, ch6:6}
        if self.parent.select:
            for key, val in self.values.items():
                if val == self.parent.select:
                    key.SetValue(1)
                    break
        ch1.Bind(wx.EVT_CHECKBOX, self.oncheck)
        ch2.Bind(wx.EVT_CHECKBOX, self.oncheck)
        ch3.Bind(wx.EVT_CHECKBOX, self.oncheck)
        ch4.Bind(wx.EVT_CHECKBOX, self.oncheck)
        ch5.Bind(wx.EVT_CHECKBOX, self.oncheck)
        ch6.Bind(wx.EVT_CHECKBOX, self.oncheck)
        button.Bind(wx.EVT_BUTTON, self.button_close_OK)
        button2.Bind(wx.EVT_BUTTON, self.button_close_Cancel)

        sb2 = wx.StaticBox(panel, wx.ID_ANY, "Spam, spam, spam", size=(370, 150))
        sb2.SetLabel("Lidar's Calibration File")
        layout2 = wx.StaticBoxSizer(sb2, wx.VERTICAL)
        layout2.Add(ch1)
        layout2.Add(ch2)
        layout2.Add(ch3)
        layout2.Add(ch4)
        layout2.Add(ch5)
        layout2.Add(ch6)
        layout.Add(layout2)

        layout3 = wx.BoxSizer(wx.HORIZONTAL)
        layout3.AddStretchSpacer()
        layout3.Add(button, 0, wx.ALL | wx.CENTER | wx.EXPAND, 4)
        layout3.Add(button2, 0, wx.ALL | wx.CENTER | wx.EXPAND, 4)
        layout3.AddStretchSpacer()
        layout.Add(layout3, 0, wx.ALIGN_CENTER, 0)
        panel.SetSizer(layout)

    def update_path(self, a):
        self.filename = self.tc2.GetValue()

    def open_dialog(self, ref):
        dlg = wx.FileDialog(self, defaultDir=self.dir, defaultFile=self.filename)
        if dlg.ShowModal() == wx.ID_OK:
            self.filename = dlg.GetDirectory() + "/" + dlg.GetFilename()
            self.tc2.SetValue(self.filename)
        dlg.Destroy()

    def oncheck(self, event):
        push = event.GetEventObject()
        if push.GetValue():
            for value in self.values.keys():
                if value == push:
                    self.select = self.values[push]
                else:
                    value.SetValue(0)
        else:
            push.SetValue(0)
            self.select = 0

    def button_close_OK(self, event):
        try:
            self.parent.file_path = self.tc2.GetValue()
            self.parent.select = self.select
            self.Close()
        finally:
            self.Destroy()

    def button_close_Cancel(self, event):
        try:
            self.parent.select = 0
            self.Close()
        finally:
            return True


if __name__ == "__main__":
	gettext.install("app")

	app = MyApp(0)
	app.MainLoop()
