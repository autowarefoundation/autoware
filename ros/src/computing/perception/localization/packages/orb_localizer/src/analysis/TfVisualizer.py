from __future__ import division
from orbndt import Pose, PoseTable
import wx
import wx.lib as wxl
import numpy as np
import multiprocessing.dummy as mp
import rospy
import tf
from matplotlib.figure import Figure, Axes
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigureCanvas



orbProc1 = None
orbProc2 = None
TIMER_ID = wx.NewId()


class TfCollector:
    def __init__ (self, parentFrameId, childFrameId, tfListener):
        self.listener = tfListener
        self.parentFrameId = parentFrameId
        self.childFrameId = childFrameId
        # Time delay before timeout
        self.timeTolerance = 0.1
        self.translation = np.zeros(3)
        self.rotation = np.zeros(4)
        self.time = -1
        self.stop = False
        self.process = mp.Process(target=self.start)
        self.process.start()
        self.updated = False

        
    def getPose (self):
        if self.time == -1:
            return None
        return Pose(self.time, \
            self.translation[0], self.translation[1], self.translation[2], \
            self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3])


    def start (self):
        print ("{} started".format(self.childFrameId))
        while (self.stop != True):
            try:
                tm = rospy.Time.now()
#                print ("Trying lookup")
                self.listener.waitForTransform (self.parentFrameId, self.childFrameId, tm, rospy.Duration(self.timeTolerance))
                (trans, rot) = self.listener.lookupTransform (self.parentFrameId, self.childFrameId, tm)
#                print ("Got ORB for {}".format(self.childFrameId))
                self.time = tm.to_sec()
                self.translation[0] = trans[0]
                self.translation[1] = trans[1]
                self.translation[2] = trans[2]
                self.rotation[0] = rot[0]
                self.rotation[1] = rot[1]
                self.rotation[2] = rot[2]
                self.rotation[3] = rot[3]
                self.updated = True
#                 print ("{}, {}, {}".format(trans[0], trans[1], trans[2]))
            except Exception:
                if self.time != -1:
                    print ("{} lost".format(self.childFrameId))
                self.time = -1


class PlotFigure (wx.Frame):
    
    def __init__ (self, groundTruth=None):
        wx.Frame.__init__ (self, None, wx.ID_ANY, title="Trajectory")
        
        self.fig = Figure ()
        self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
        self.ax = self.fig.add_subplot (111)
        self.ax.set_xlim ([-600, 1000])
        self.ax.set_ylim ([-1500, 1500])
        self.ax.set_autoscale_on (False)
        self.orbPos1 = None
        self.orbPos2 = None
        self.ax.grid(True)
        
        if groundTruth != None:
            grnd = groundTruth.toArray(False)
            self.groundPlot, = self.ax.plot (grnd[:,0], grnd[:,1])

        # This must be done after all initial drawing
        self.canvas.draw()
        self.bg = self.canvas.copy_from_bbox (self.ax.bbox)
        
        # Bind events to timer function
        wx.EVT_TIMER (self, TIMER_ID, self.onTimer)

    def onTimer (self, event):
        self.canvas.restore_region(self.bg)
        
        orbPosition1 = orbProc1.getPose()
        if orbPosition1 is not None:
            if self.orbPos1 is None:
                self.orbPos1 = self.ax.scatter (orbPosition1.x, orbPosition1.y, color=[[1,0,0,0.5]], s=100, linewidths=0)
            else :
                self.orbPos1.set_offsets([orbPosition1.x, orbPosition1.y])

        orbPosition2 = orbProc2.getPose()
        if orbPosition2 is not None:
            if self.orbPos2 is None:
                self.orbPos2 = self.ax.scatter (orbPosition2.x, orbPosition2.y, color=[[0,1,0,0.5]], s=100, linewidths=0)
            else :
                self.orbPos2.set_offsets([orbPosition2.x, orbPosition2.y])


        self.canvas.draw()
        self.canvas.blit(self.ax.bbox)
        
        
        

class VisualizerApp (wx.PySimpleApp):
    def OnInit (self):
        self.frame = AnalyzerWindow()
        self.frame.Show(True)
        self.SetTopWindow(self.frame)
        return True


class tfBagVisualizer (wx.Frame):
    
    def __init__ (self, bag):
        pass
    
    
if __name__ == '__main__':
    
    from sys import argv
    
    groundTruth = PoseTable.loadFromBagFile (argv[1], 'world', 'camera1')
    app = wx.PySimpleApp()
    frame = PlotFigure(groundTruth)
    timer = wx.Timer(frame, TIMER_ID)
    timer.Start(50)
    
    rospy.init_node('ORB_Listener', anonymous=True)
    orbListener = tf.TransformListener()
    
    orbProc1 = TfCollector ('world', 'camera1', orbListener)
    orbProc2 = TfCollector ('world', 'camera2', orbListener)
    
    frame.Show()
    app.MainLoop()
    
    orbProc1.stop = True
    sleep (0.5)