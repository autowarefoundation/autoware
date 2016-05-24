#!/usr/bin/env python
#-*- coding: utf-8 -*-

import wx
import sys
import re
import multiprocessing
import socket
import select
import yaml
import pickle
#import hashlib
import rosnode
import rosgraph
try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

class MyFrame(wx.Frame):
  def __init__(self, parent, id, title):
    wx.Frame.__init__(self, parent, id, title)
    panel = wx.Panel(self, -1)
    vbox = wx.BoxSizer(wx.VERTICAL)
    panel.SetSizer(vbox)
    self.hpanel_ = wx.Panel(panel, -1)
    self.hbox_ = wx.BoxSizer(wx.HORIZONTAL)
    self.hpanel_.SetSizer(self.hbox_)
    self.clpanel_ = wx.Panel(self.hpanel_, -1)
    self.clbox_ = wx.BoxSizer(wx.VERTICAL)
    self.clpanel_.SetSizer(self.clbox_)
    self.hbox_.Add(self.clpanel_, flag=wx.ALL, border=2)
    self.cgpanel_ = wx.Panel(self.hpanel_, -1)
    self.cgbox_ = wx.BoxSizer(wx.VERTICAL)
    self.cgpanel_.SetSizer(self.cgbox_)
    self.hbox_.Add(self.cgpanel_, flag=wx.ALL, border=2)
    self.labels_ = []
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
    vbox.Add(self.hpanel_, flag=wx.ALL, border=2)

    self.lpanel_.SetSizer(self.lbox_)
    vbox.Add(self.lpanel_, wx.ALL, border=8)

    spanel = wx.Panel(panel, -1)
    hbox = wx.BoxSizer(wx.HORIZONTAL)
    sbtn = wx.Button(spanel, -1, "update")
    sbtn.Bind(wx.EVT_BUTTON, self.OnUpdate)
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
    self.clbox_.Clear(True)
    self.cgbox_.Clear(True)
    self.lbox_.Clear(True)
    self.lbox_.Add(wx.StaticText(self.lpanel_, -1, "  "), flag=wx.ALL, border=2)

    # add
    self.labels_ = []
    if self.view_ == 0:
      for cpuno in range(0, self.cpucount_):
        text = wx.StaticText(self.clpanel_, -1, "CPU%02d:" % (cpuno))
        self.clbox_.Add(text, flag=wx.ALIGN_RIGHT|wx.ALL, border=4)
        spc = wx.StaticText(self.cgpanel_, -1, " "*160)
        self.cgbox_.Add(spc, flag=wx.ALL, border=4)
        self.labels_.append(cpuno)
      for (name, pid) in self.pids_:
        text = wx.StaticText(self.lpanel_, -1, u"■%d" % (pid))
        text.SetForegroundColour(self.GetColor(0, pid))
        text.SetToolTip(wx.ToolTip(name))
        self.lbox_.Add(text, flag=wx.ALL, border=2)
    else:
      for (name, pid) in self.pids_:
        text = wx.StaticText(self.clpanel_, -1, "%s:" % (name))
        text.SetToolTip(wx.ToolTip("pid=%d" % pid))
        self.clbox_.Add(text, flag=wx.ALIGN_RIGHT|wx.ALL, border=4)
        spc = wx.StaticText(self.cgpanel_, -1, " "*160)
        self.cgbox_.Add(spc, flag=wx.ALL, border=4)
        self.labels_.append(pid)
      for cpuno in range(0, self.cpucount_):
        text = wx.StaticText(self.lpanel_, -1, u"■CPU%02d" % (cpuno))
        text.SetForegroundColour(self.GetColor(cpuno, 0))
        self.lbox_.Add(text, flag=wx.ALL, border=2)
    self.clbox_.Layout()
    self.cgbox_.Layout()
    self.lbox_.Layout()
    if fit:
      self.hpanel_.GetParent().Fit() # panel
      self.hpanel_.GetParent().GetParent().Fit() # frame

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

  def _prv_recv(self, sock, dlen):
    ret = sock.recv(dlen)
    rlen = len(ret)
    while rlen < dlen:
      (r, _, _) = select.select([sock], [], [], 0.2)
      if len(r) <= 0:
        break
      d = sock.recv(dlen - rlen)
      if len(d) <= 0:
        break
      ret += d
      rlen += len(d)
    return ret

  def OnUpdate(self, event):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
      sock.connect("/tmp/autoware_proc_manager")
    except socket.error:
      print 'Error: cannot connect to proc_manager...'
      return
    order = { 'name':'ftrace', 'sec':1 }
    sock.send(yaml.dump(order))
    dat = self._prv_recv(sock, 16*1024*1024)
    sock.close()
    #print '# recv %d, md5sum %s' % (len(dat), hashlib.md5(dat).hexdigest())
    dat = pickle.loads(dat)
    tmax = 0
    for cpuno in dat:
      for d in dat[cpuno]:
        if d[1] > tmax:
          tmax = d[1]
    self.UpdateGraph(dat, tmax)

  def OnChangeView(self, event):
    self.view_ = 1 if self.view_ == 0 else 0
    self.ChangeView(True)

  def ClearGraph(self):
    n = 0
    for cpuno in self.labels_:
      col = self.cgpanel_.GetBackgroundColour()
      spc = self.cgpanel_.GetChildren()[n]
      pos = spc.GetPosition()
      rect = spc.GetRect()
      dc = wx.PaintDC(self.cgpanel_)
      dc.SetPen(wx.Pen(col))
      dc.SetBrush(wx.Brush(col))
      dc.DrawRectangle(pos.x, pos.y, rect.width, rect.height)
      n += 1

  def UpdateGraph(self, dat, tmax):
    self.ClearGraph()
    for cpuno in range(0, self.cpucount_):
      ptm = 0
      pid = 0
      for d in dat[cpuno]:
        cid = d[0]
        ctm = d[1]
        col = ''
        for (_, p) in self.pids_:
          if p == pid:
            col = self.GetColor(cpuno, pid)
            break
        if len(col) == 0:
          if self.view_ == 0 and pid != 0:
            col = '#cccccc'
          else:
            ptm = ctm
            pid = cid
            continue
        n = self.labels_.index(cpuno if self.view_ == 0 else pid)
        spc = self.cgpanel_.GetChildren()[n]
        pos = spc.GetPosition()
        rect = spc.GetRect()
        dc = wx.PaintDC(self.cgpanel_)
        dc.SetPen(wx.Pen(col))
        dc.SetBrush(wx.Brush(col))
        w = rect.width*(ctm-ptm)/tmax + 2
        dc.DrawRectangle(pos.x + rect.width*ptm/tmax, pos.y, w, rect.height)
        ptm = ctm
        pid = cid

  def GetColor(self, cpuno, pid):
    if self.view_ == 0:
      i = 0
      for (_, p) in self.pids_:
        if p == pid:
          return self.colors_[i % len(self.colors_)]
        i += 1
      return self.colors_[0]
    else:
      return self.colors_[cpuno % len(self.colors_)]

class MyApp(wx.App):
  def OnInit(self):
    self.frame_ = MyFrame(None, -1, "Ftrace")
    self.SetTopWindow(self.frame_)
    return True

  def SetPids(self, pids):
    self.frame_.SetPids(pids)

def getRosNodes():
  nodes = []
  try:
    nodenames = rosnode.get_node_names(None)
  except Exception as inst:
    print "Error:", inst
    sys.exit(2)
  for nodename in nodenames:
    #rosnode.rosnode_info(nodename)
    api = rosnode.get_api_uri(rosgraph.Master('/rosnode'), nodename)
    if api:
      try:
        node = ServerProxy(api)
        code, msg, pid = node.getPid('/rosnode')
        if code == 1:
          res = re.search('^(.*)_[0-9]+_[0-9]+$', nodename)
          if res is None:
            nodes.append((nodename, pid))
          else:
            nodes.append((res.group(1), pid))
      except:
        pass
  return nodes

if __name__ == "__main__":
  app = MyApp(0)
  rosnodes = getRosNodes()
  #print '# nodes:', rosnodes
  app.SetPids(rosnodes)
  app.MainLoop()
