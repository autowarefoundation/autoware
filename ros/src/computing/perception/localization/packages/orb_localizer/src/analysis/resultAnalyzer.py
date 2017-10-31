#!/usr/bin/python
# ORB-MAP Result Analyzer
#

from __future__ import division
from orbndt import PoseTable
import datetime
import wx
import wx.lib as wxl
import numpy as np
from matplotlib.figure import Figure, Axes
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigureCanvas
from math import sin, cos, tan
import rosbag
import rospy
import yaml
from copy import copy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import cv2




class AnalyzerApp (wx.PySimpleApp):
    def OnInit (self):
        self.frame = AnalyzerWindow()
        self.frame.Show(True)
        self.SetTopWindow(self.frame)
        return True



class AnalyzerWindow (wx.Frame):
    
    def __init__ (self, *args, **kwargs):
        wx.Frame.__init__ (self, None, wx.ID_ANY)
        self.SetTitle('ORB-SLAM Analyzer')
        
        #Set-up menu
        menubar = wx.MenuBar()
        fileMenu = wx.Menu()
        analyzeResultItem = fileMenu.Append(wx.ID_ANY, 'Analyze Result')
        analyzeMapItem = fileMenu.Append(wx.ID_ANY, 'Analyze Map')
        qitem = fileMenu.Append(wx.ID_EXIT, 'Quit', 'Quit Analyzer')
        
        menubar.Append(fileMenu, '&File')
        self.SetMenuBar(menubar)

        self.Bind(wx.EVT_MENU, self.onQuit, qitem)
        self.Bind(wx.EVT_MENU, self.onClickAnalyzeMap, analyzeMapItem)
        self.Bind(wx.EVT_MENU, self.onClickAnalyzeResult, analyzeResultItem)
        
        self.Centre()
#        self.SetSize((800,600))
        
    def onQuit(self, e):
        self.Close()
        
    def onClickAnalyzeResult(self, e):
        p = AnalyzeORBResult(self)
#        l = wx.BoxSizer(wx.VERTICAL)
#        l.Add(p, flag=wx.EXPAND)
#        self.SetSizer(l)
        
#        self.AddChild(p)
        self.Layout()
        self.Fit()
    
    def onClickAnalyzeMap (self, e):
        askFile = wx.FileDialog(self, "Choose file")
        if askFile.ShowModal() == wx.ID_CANCEL:
            print ("Cancelled")
        print (askFile.GetPath())
        

class FilePrompt (wx.BoxSizer):
    def __init__ (self, parent, text, handler=None):
        self.prompt = text
        self.parent = parent
        self.handler = handler
        super(FilePrompt, self).__init__(wx.HORIZONTAL)
        self.Add(wx.StaticText(parent, label=text), flag=wx.ALIGN_LEFT|wx.ALIGN_CENTER_VERTICAL, proportion=0)
        self.input = wx.TextCtrl(parent, style=wx.TE_READONLY)
        self.Add(self.input, flag=wx.EXPAND|wx.ALIGN_CENTER, proportion=1)
        btnCh = wx.Button(parent)
        btnCh.Bind(wx.EVT_BUTTON, self.onClickChange)
        self.Add(btnCh, flag=wx.ALIGN_RIGHT, proportion=0)
        
    def onClickChange (self, e):
        askFile = wx.FileDialog(self.parent, )
        if askFile.ShowModal() == wx.ID_CANCEL:
            return
        self.input.SetValue(askFile.GetPath())
        if self.handler!=None:
            self.handler(self.input.GetValue())
            
    def GetValue(self):
        return self.input.GetValue()
        
        
class BagImagePreview (wx.Panel):
    def __init__ (self, parent, bagSourcePath=None, topic=None):
        wx.Panel.__init__ (self, parent)
        if (bagSourcePath!=None):
            self.setBagPath (bagSourcePath)
        self.setTopic (topic)
        self.imgConverter = CvBridge()
#        self.bitmap = wx.Bitmap()
        
    def setBagPath (self, bagSourcePath):
        self.bagsrc = rosbag.Bag(bagSourcePath, mode='r')
        self.bagTopics = yaml.load(self.bagsrc._get_yaml_info('topics'))
        self.bagTopicDict = {}
        for t in self.bagTopics:
            self.bagTopicDict[t['topic']] = copy(t)
        
        
    def setTopic (self, topic):
        self.topic = topic

    def getAllTopics (self):
        ret = []
        for t in self.bagTopics:
            ret.append(t['topic'])
        return ret
        
    def showTime (self, timestamp):
        msg = BagImagePreview.readMessage (self.bagsrc, self.topic, timestamp)
        image = self.imgConverter.imgmsg_to_cv2 (msg, 'bgr8')
        imgWidth, imgHeight = image.shape[1], image.shape[0]
        # XXX: Unfinished
        # 1: Determine appropriate size
        width, height = self.GetSize()
        height = imgHeight * (width / imgWidth)
        self.SetSize ((width, height))
        # 2: Resize Image
        image = cv2.resize(image, (height,width))
        # 3: Draw
        self.bitmap = wx.BitmapFromBuffer(width, height, image)
        self.Refresh()
        
    def onPaint (self, evt):
        dc = wx.BufferedPaintDC (self)
        dc.DrawBitmap (self.bitmap, 0, 0)
    
    @staticmethod
    def readMessage (bag, topic, timestamp):
        tm = rospy.Time.from_sec(timestamp)
        for topic, msg, time in bag.read_messages(topics=topic, start_time=tm):
            if msg is None:
                print('Not found')
                continue
            return copy(msg)


        
class AnalyzeORBResult (wx.Panel) :

    def __init__ (self, parent):
        wx.Panel.__init__(self, parent=parent)
        
        splitter = wx.SplitterWindow (self)
        
        # XXX: we may need to place the canvas in its own Panel
        self.figure = Figure()
        self.canvas = FigureCanvas(splitter, wx.ID_ANY, self.figure)
        self.vehiclePointer = None
        self.imageTopic = None

        rightPanel = wx.Panel(splitter)
        cPanel = wx.BoxSizer (wx.VERTICAL)
        rightPanel.SetSizer(cPanel)

        self.orbResultChooser = FilePrompt(rightPanel, 'ORB-SLAM Result Bag', self.onClickOrbChoose)
        cPanel.Add(self.orbResultChooser, flag=wx.EXPAND)
        
        self.groundTruthChooser = FilePrompt(rightPanel, 'GroundTruth', self.onClickGtChoose)
        cPanel.Add(self.groundTruthChooser, flag=wx.EXPAND)

        timePosBox = wx.BoxSizer(wx.HORIZONTAL)        
        timePosBox.Add(wx.StaticText(rightPanel, label='Time Position'), flag=wx.ALIGN_LEFT|wx.ALIGN_CENTER_VERTICAL, proportion=0)
        self.timeDisplay = wx.TextCtrl(rightPanel, style=wx.TE_READONLY)
        timePosBox.Add(self.timeDisplay, flag=wx.EXPAND|wx.ALIGN_CENTER, proportion=1)
        cPanel.Add(timePosBox, flag=wx.EXPAND)

        orbStatusBox = wx.BoxSizer(wx.HORIZONTAL)        
        orbStatusBox.Add(wx.StaticText(rightPanel, label="ORB Status"), flag=wx.ALIGN_LEFT|wx.ALIGN_CENTER_VERTICAL, proportion=0)
        self.orbStatus = wx.StaticText(rightPanel, label="Init")
        orbStatusBox.Add(self.orbStatus, flag=wx.ALIGN_CENTER|wx.EXPAND, proportion=1)
        cPanel.Add(orbStatusBox, flag=wx.EXPAND)

        bagGroup = wx.StaticBoxSizer(wx.StaticBox(rightPanel, label='Image Source Bag'), wx.VERTICAL)
        askBag = FilePrompt(rightPanel, "Image Bag File", self.onBagChange)
        bagGroup.Add(askBag, flag=wx.EXPAND)
        self.imageTopicChooser = wx.ComboBox(rightPanel)
        bagGroup.Add(self.imageTopicChooser, flag=wx.EXPAND)
        self.imageTopicChooser.Bind(wx.EVT_COMBOBOX, self.onImageTopicChange)
        self.bagImageView = BagImagePreview (rightPanel)
        bagGroup.Add (self.bagImageView, flag=wx.EXPAND)
        cPanel.Add(bagGroup, flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL)

        splitter.SplitVertically(self.canvas, rightPanel)
        splitter.SetMinimumPaneSize(30)

        mainLayout = wx.BoxSizer (wx.VERTICAL)
        self.SetSizer(mainLayout)  
        mainLayout.Add(splitter, proportion=1, flag=wx.EXPAND|wx.ALIGN_TOP)  
        
        self.timeChooser = wx.Slider(self)
        self.timeChooser.Bind(wx.EVT_SCROLL, self.onChangeTimePosition)
        mainLayout.Add(self.timeChooser, flag=wx.EXPAND|wx.ALIGN_BOTTOM)
        self.Fit()
        
    
    def onClickOrbChoose (self, bagPath):
        self.loadData()
    
    def onClickGtChoose (self, gtPath):
        self.loadData()
        
    def onChangeTimePosition (self, e):
        if self.currentORBTimestamp == None:
            return
#        print (self.timeChooser.GetValue())
        self.currentORBTimestamp = int(self.timeChooser.GetValue())
        self.redrawPosition()
        
    def onBagChange (self, bagPath):
        self.bagImageView.setBagPath (bagPath)
        self.imageTopicChooser.AppendItems(self.bagImageView.getAllTopics())
    
    def onImageTopicChange (self, e):
        self.imageTopic = (self.imageTopicChooser.GetValue())
        self.bagImageView.setTopic (self.imageTopic)
        self.redrawPosition()
        
        
    def loadData (self):
        if (self.orbResultChooser.GetValue()=='' or self.groundTruthChooser.GetValue()==''):
            print ("Select filenames first")
            return None
        print ("Wait...")
        
        # Load ground truth and plot it
        self.ax = self.figure.add_subplot(111)
        self.ax.set_autoscale_on(True)
        self.ax.grid(True)
        self.groundTruth = PoseTable.loadFromBagFile(self.groundTruthChooser.GetValue(), 'world', 'ndt_frame')
        gtTbl = self.groundTruth.toArray()
        self.groundTruthPlot, = self.ax.plot(gtTbl[:,0], gtTbl[:,1])
#        self.groundTruthPlot, = self.ax.plot([0,1,2], [4,5,6])
        
        self.orbResult = PoseTable.loadFromBagFile(self.orbResultChooser.GetValue(), '/ORB_SLAM/World', '/ORB_SLAM/ExtCamera')
        orbTbl = self.orbResult.toArray()
        self.orbResultPlot, = self.ax.plot(orbTbl[:,0], orbTbl[:,1])
        
        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
#        self.helpText.Show(True)
        
        self.timeChooser.SetRange(self.orbResult[0].timestamp, self.orbResult.last().timestamp)
        self.currentORBTimestamp = self.orbResult[0].timestamp
        self.redrawPosition()
        
    @staticmethod
    def readMessage (bag, topic, timestamp):
        tm = rospy.Time.from_sec(timestamp)
        for topic, msg, time in bag.read_messages(topics=topic, start_time=tm):
            return msg
        
    def redrawPosition (self):
        if self.currentORBTimestamp == None:
            return
        dp = datetime.datetime.fromtimestamp(self.currentORBTimestamp)
        self.timeDisplay.SetValue(str(dp))
        
        orbPose = self.orbResult.findNearestInTime (self.currentORBTimestamp, 0.1)
        
        self.canvas.restore_region(self.background)
        if (orbPose!=None):
            if (self.vehiclePointer==None):
                self.vehiclePointer = self.ax.scatter(orbPose.x, orbPose.y, s=100, c=[1,0,0,0.5], linewidths=0)
            else:
                self.vehiclePointer.set_offsets([orbPose.x, orbPose.y])
            self.orbStatus.SetLabel("OK")
        else:
            self.orbStatus.SetLabel("Lost")
        self.canvas.draw()
        self.canvas.blit(self.ax.bbox)
        
        if self.imageTopic != None:
            self.bagImageView.showTime (self.currentORBTimestamp)
            # XXX: how to display image
        

if __name__ == '__main__' :
    app = AnalyzerApp()
    app.MainLoop()
#    bag=rosbag.Bag('/media/sujiwo/ssd/log_2016-01-27-11-04-42.bag', mode='r')
#    msg=AnalyzeORBResult.readMessage(bag, '/camera1/image_raw', 1453861282)
#    print('XXX')

