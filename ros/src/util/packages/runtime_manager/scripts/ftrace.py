#!/usr/bin/env python
#-*- coding: utf-8 -*-

import wx
import sys
import multiprocessing
import re
import time
import select
import threading

class SchedSwitchThread(threading.Thread):
  def __init__(self):
    super(SchedSwitchThread, self).__init__()
    self.toff_ = 0
    self.tmax_ = 0
    self.running_ = True

  def stop(self, timeout=None):
    self.running_ = False
    self.join(timeout)

  def run(self):
    self.dat_ = {}
    # skip old data
    f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
    self.doCaptureOnce(f, {})
    f.close()
    self.doEnableSchedSwitch(True)
    self.doTrace(True)
    stime = time.time()
    f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
    self.toff_ = 0
    while self.running_:
      self.doCaptureOnce(f, self.dat_)
      time.sleep(0.05)
    self.doTrace(False)
    self.doEnableSchedSwitch(False)
    self.doCaptureOnce(f, self.dat_)
    f.close()

  def doEnableSchedSwitch(self, t):
    f = open('/sys/kernel/debug/tracing/events/sched/sched_switch/enable', 'w')
    f.write('1' if t else '0')
    f.close()

  def doTrace(self, t):
    f = open('/sys/kernel/debug/tracing/tracing_on', 'w')
    f.write('1' if t else '0')
    f.close()

  def doCaptureOnce(self, f, ret):
    dt = 0
    while True:
      (r, _, _) = select.select([f], [], [], 0)
      if len(r) <= 0:
        break
      l = f.readline()
      m = re.match('^.* \[([0-9]*)\].* ([0-9]*\.[0-9]*): .*==> next_comm=(.*) next_pid=([0-9]*) next.*$', l)
      if m is None:
        continue
      dt = float(m.group(2))
      if self.toff_ == 0:
        self.toff_ = dt
      dt -= self.toff_
      cpuno = int(m.group(1))
      pid = int(m.group(4))
      d = (dt, pid, m.group(3))
      if cpuno not in ret:
        ret[cpuno] = []
      ret[cpuno].append(d)
    if self.tmax_ < dt:
      self.tmax_ = dt

  def doCapture(self, sec):
    self.dat_ = {}
    # skip old data
    f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
    self.doCaptureOnce(f, {})
    f.close()

    self.doEnableSchedSwitch(True)
    self.doTrace(True)
    stime = time.time()
    time.sleep(sec)
    self.doTrace(False)
    self.doEnableSchedSwitch(False)

    f = open('/sys/kernel/debug/tracing/trace_pipe', 'r')
    self.toff_ = 0
    self.doCaptureOnce(f, self.dat_)
    f.close()

class MyFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title)
    panel = wx.Panel(self, -1)
    vbox = wx.BoxSizer(wx.VERTICAL)
    panel.SetSizer(vbox)
    self.vpanel_ = wx.Panel(panel, -1)
    self.vbox_ = wx.BoxSizer(wx.VERTICAL)
    self.vpanel_.SetSizer(self.vbox_)
    self.panels_ = {}
    self.cpucount_ = multiprocessing.cpu_count()
    self.sec_ = 1
    self.pids_ = []
    self.view_ = 0 # CPU
    self.colors_ = [
      '#cc0000', '#00cc00', '#0000cc',
      '#cccc00', '#cc00cc', '#00cccc',
      '#990000', '#009900', '#000099',
      '#999900', '#990099', '#009999',
      '#660000', '#006600', '#000066',
      '#666600', '#660066', '#006666',
      '#330000', '#003300', '#000033',
      '#333300', '#330033', '#003333']

    self.lpanel_ = wx.Panel(panel, -1)
    self.lbox_ = wx.BoxSizer(wx.HORIZONTAL)

    self.ChangeView(False)
    vbox.Add(self.vpanel_, flag=wx.ALL, border=2)

    self.lpanel_.SetSizer(self.lbox_)
    vbox.Add(self.lpanel_, wx.ALL, border=8)

    spanel = wx.Panel(panel, -1)
    hbox = wx.BoxSizer(wx.HORIZONTAL)
    #self.text_ = wx.TextCtrl(spanel, -1, str(self.sec_))
    #hbox.Add(self.text_, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    #hbox.Add(wx.StaticText(spanel, -1, " sec  "),
    #  flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    #cbtn = wx.Button(spanel, -1, "capture")
    #cbtn.Bind(wx.EVT_BUTTON, self.OnClick)
    #hbox.Add(cbtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    sbtn = wx.Button(spanel, -1, "start")
    sbtn.Bind(wx.EVT_BUTTON, self.OnStartStop)
    hbox.Add(sbtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    ebtn = wx.Button(spanel, -1, "close")
    vbtn = wx.Button(spanel, -1, "cpu/proc")
    vbtn.Bind(wx.EVT_BUTTON, self.OnChangeView)
    hbox.Add(vbtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    ebtn.Bind(wx.EVT_BUTTON, self.OnExit)
    hbox.Add(ebtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)

    spanel.SetSizer(hbox)
    vbox.Add(spanel, flag=wx.ALL, border=10)

    self.SetSize(panel.GetBestSize())
    self.Centre()
    self.Show(True)

  def ChangeView(self, fit):
    # remove
    self.vbox_.Clear(True)
    self.lbox_.Clear(True)
    self.lbox_.Add(wx.StaticText(self.lpanel_, -1, "  "), flag=wx.ALL, border=2)
    del self.panels_

    # add
    self.panels_ = {}
    if self.view_ == 0:
      for cpuno in range(0, self.cpucount_):
        spanel = wx.Panel(self.vpanel_, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(spanel, -1, "CPU%02d: " % (cpuno)))
        spc = wx.StaticText(spanel, -1, " "*160)
        hbox.Add(spc)
        spanel.SetSizer(hbox)
        self.vbox_.Add(spanel, flag=wx.ALL, border=10)
        self.panels_[cpuno] = spanel
      for pid in self.pids_:
        text = wx.StaticText(self.lpanel_, -1, u"■%05d" % (pid))
        text.SetForegroundColour(self.GetColor(0, pid))
        self.lbox_.Add(text, flag=wx.ALL, border=2)
    else:
      for pid in self.pids_:
        spanel = wx.Panel(self.vpanel_, -1)
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        hbox.Add(wx.StaticText(spanel, -1, "%05d: " % (pid)))
        spc = wx.StaticText(spanel, -1, " "*160)
        hbox.Add(spc)
        spanel.SetSizer(hbox)
        self.vbox_.Add(spanel, flag=wx.ALL, border=10)
        self.panels_[pid] = spanel
      for cpuno in range(0, self.cpucount_):
        text = wx.StaticText(self.lpanel_, -1, u"■CPU%02d" % (cpuno))
        text.SetForegroundColour(self.GetColor(cpuno, 0))
        self.lbox_.Add(text, flag=wx.ALL, border=2)
    self.vbox_.Layout()
    self.lbox_.Layout()
    if fit:
      self.vpanel_.GetParent().Fit() # panel
      self.vpanel_.GetParent().GetParent().Fit() # frame

  def SetPids(self, pids):
    self.pids_ = pids
    self.ChangeView(True)

  def OnExit(self, event):
    sys.exit(0)

  def OnText(self, event):
    s = self.text_.GetValue()
    if s.isdigit():
      val = int(s)
      if val > 0:
        self.sec_ = val
    self.text_.SetValue(str(self.sec_))

  def OnClick(self, event):
    self.OnText(None)
    ss = SchedSwitchThread()
    dat = ss.doCapture(self.sec_)
    self.UpdateGraph(ss)

  def OnStartStop(self, event):
    btn = event.GetEventObject()
    if btn.GetLabel() == "start":
      btn.SetLabel("stop")
      self.ss_ = SchedSwitchThread()
      self.ss_.start()
    else:
      btn.SetLabel("start")
      self.ss_.stop()
      self.UpdateGraph(self.ss_)

  def OnChangeView(self, event):
    self.view_ = 1 if self.view_ == 0 else 0
    self.ChangeView(True)

  def ClearGraph(self):
    for cpuno in self.panels_.keys():
      panel = self.panels_[cpuno]
      col = panel.GetBackgroundColour()
      spc = panel.GetChildren()[1]
      pos = spc.GetPosition()
      rect = spc.GetRect()
      dc = wx.PaintDC(panel)
      dc.SetPen(wx.Pen(col))
      dc.SetBrush(wx.Brush(col))
      dc.DrawRectangle(pos.x, pos.y, rect.width, rect.height)

  def UpdateGraph(self, ss):
    self.ClearGraph()
    for cpuno in range(0, self.cpucount_):
      ptm = 0
      pid = 0
      for (ctm, cid, _) in ss.dat_[cpuno]:
        if pid in self.pids_:
          col = self.GetColor(cpuno, pid)
        elif self.view_ == 0 and pid != 0:
          col = '#cccccc'
        else:
          ptm = ctm
          pid = cid
          continue
        panel = self.panels_[cpuno if self.view_ == 0 else pid]
        spc = panel.GetChildren()[1]
        pos = spc.GetPosition()
        rect = spc.GetRect()
        dc = wx.PaintDC(panel)
        dc.SetPen(wx.Pen(col))
        dc.SetBrush(wx.Brush(col))
        w = rect.width*(ctm-ptm)/ss.tmax_ + 2
        dc.DrawRectangle(pos.x + rect.width*ptm/ss.tmax_, pos.y, w, rect.height)
        ptm = ctm
        pid = cid

  def GetColor(self, cpuno, pid):
    return self.colors_[(self.pids_.index(pid) if self.view_ == 0 else cpuno) % len(self.colors_)]

class MyApp(wx.App):
  def OnInit(self):
    self.frame_ = MyFrame(None, -1, "sched_switch graph")
    self.SetTopWindow(self.frame_)
    return True

  def SetPids(self, pids):
    self.frame_.SetPids(pids)

if __name__ == "__main__":
  app = MyApp(0)
  app.SetPids(map(lambda n:int(n), sys.argv[1:]))
  app.MainLoop()
