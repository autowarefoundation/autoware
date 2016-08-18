#!/usr/bin/env python
#-*- coding: utf-8 -*-

import wx
import sys
import datetime
import multiprocessing
import pickle
import re
import socket
import select
import yaml
import rosnode
import rosgraph
try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

# メインフレームクラス
class MyFrame(wx.Frame):
  # MyFrame: 初期画面表示関数
  def __init__(self, parent, id, title):
    # 基底パネルの設定
    wx.Frame.__init__(self, parent, id, title)
    panel = wx.Panel(self, -1)
    vbox = wx.BoxSizer(wx.VERTICAL)
    panel.SetSizer(vbox)

    # 上部バネル(グラフ表示部)の設定
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
    vbox.Add(self.hpanel_, flag=wx.ALL, border=2)

    # 中部バネル(コメント表示)の設定
    self.lpanel_ = wx.Panel(panel, -1)
    self.lbox_ = wx.BoxSizer(wx.HORIZONTAL)
    self.lpanel_.SetSizer(self.lbox_)
    vbox.Add(self.lpanel_, wx.ALL, border=8)

    # 下部バネル(操作ボタン表示)の設定
    spanel = wx.Panel(panel, -1)
    hbox = wx.BoxSizer(wx.HORIZONTAL)
    self.sbtn = wx.Button(spanel, -1, "Start/Stop")
    self.sbtn.Bind(wx.EVT_BUTTON, self.OnUpdateCont)
    hbox.Add(self.sbtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    vbtn = wx.Button(spanel, -1, "CPU/Node")
    vbtn.Bind(wx.EVT_BUTTON, self.OnChangeView)
    hbox.Add(vbtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    ebtn = wx.Button(spanel, -1, "Close")
    ebtn.Bind(wx.EVT_BUTTON, self.OnExit)
    hbox.Add(ebtn, flag=wx.ALIGN_CENTER_VERTICAL|wx.ALL, border=2)
    spanel.SetSizer(hbox)
    vbox.Add(spanel, flag=wx.ALL, border=10)

    # 初期値設定
    self.labels_ = []
    self.cgpanels_ = []
    self.cpucount_ = multiprocessing.cpu_count()
    self.pids_ = []
    self.view_ = 0 # 0=CPUno, 1=ROSnode(pid)
    self.itmcount_ = 0 # 表示アイテム数（時刻欄を除く）
    self.itmcolors_ = [
      '#cc0000', '#00cc00', '#0000cc',
      '#cccc00', '#cc00cc', '#00cccc',
      '#990000', '#009900', '#000099',
      '#999900', '#990099', '#009999',
      '#660000', '#006600', '#000066',
      '#666600', '#660066', '#006666',
      '#330000', '#003300', '#000033',
      '#333300', '#330033', '#003333']
    self.bgcol_ = '#ffffff' # 描画背景色
    self.cgtw_, self.cgth_ = wx.StaticText(self.cgpanel_, -1, " "*320).GetSize() # 描画欄の幅と高さの取得

    # 画面表示
    self.ChangeView(False)
    self.SetSize(panel.GetBestSize())
    self.Centre()
    self.Show(True)

    # タイマ準備
    self.timer = wx.Timer(self)
    self.Bind(wx.EVT_TIMER, self.OnTimer)

    # 制御変数の初期設定
    self.update = False # 1=update executing
    self.sock = None # comm with proc_manager.py
    self.dtimespan = 3.0 # 描画されるデータの時間 [s]
    self.dtime = 0 # 描画されたデータの最新の時刻 [s]

  # MyFrame: CPUno／ROSnode 表示形態変更関数
  def ChangeView(self, fit):
    # 今までのフォーマットを消去
    self.clbox_.Clear(True)
    self.cgbox_.Clear(True)
    self.lbox_.Clear(True)
    self.lbox_.Add(wx.StaticText(self.lpanel_, -1, "  "), flag=wx.ALL, border=2)
    self.labels_ = []
    self.cgpanels_ = []
    self.pids_ = self.getRosNodes()

    # 新しいフォーマットに描き直し
    if self.view_ == 0:
      self.itmcount_ = self.cpucount_
      for cpuno in range(0, self.cpucount_):
        text = wx.StaticText(self.clpanel_, -1, "CPU%02d:" % cpuno)
        self.clbox_.Add(text, flag=wx.ALIGN_RIGHT|wx.ALL, border=4)
        self.labels_.append(cpuno)

      for (name, pid) in self.pids_:
        text = wx.StaticText(self.lpanel_, -1, u"■%d" % pid)
        text.SetForegroundColour(self.GetColor(0, pid))
        text.SetToolTip(wx.ToolTip(name))
        self.lbox_.Add(text, flag=wx.ALL, border=2)
    else:
      self.itmcount_ = len(self.pids_)
      for (name, pid) in self.pids_:
        text = wx.StaticText(self.clpanel_, -1, "%s:" % name)
        text.SetToolTip(wx.ToolTip("pid=%d" % pid))
        self.clbox_.Add(text, flag=wx.ALIGN_RIGHT|wx.ALL, border=4)
        self.labels_.append(pid)

      for cpuno in range(0, self.cpucount_):
        text = wx.StaticText(self.lpanel_, -1, u"■CPU%02d" % cpuno)
        text.SetForegroundColour(self.GetColor(cpuno, 0))
        self.lbox_.Add(text, flag=wx.ALL, border=2)

    text = wx.StaticText(self.clpanel_, -1, "Time:") # 時刻欄を追加
    self.clbox_.Add(text, flag=wx.ALIGN_RIGHT|wx.ALL, border=4)

    for n in range(0, self.itmcount_ + 1):
      panel = wx.Panel(self.cgpanel_, -1, size=(self.cgtw_, self.cgth_))
      panel.SetBackgroundColour(self.bgcol_)
      self.cgbox_.Add(panel, flag=wx.ALL, border=4)
      self.cgpanels_.append(panel)

    self.clbox_.Layout()
    self.cgbox_.Layout()
    self.lbox_.Layout()
    if fit:
      self.hpanel_.GetParent().Fit() # panel
      self.hpanel_.GetParent().GetParent().Fit() # frame

  # MyFrame: updateボタンによるftrace処理開始／停止処理関数
  def OnUpdateCont(self, event):
    if self.update == False:
      # proc_managerに接続
      self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
      try:
        self.sock.connect("/tmp/autoware_proc_manager")
      except socket.error:
        print "ERROR:[OnUpdateCont-01] cannot connect to proc_manager"
        return

      try:
        order = { "name":"ftrace_cont", "interval":0.1, "pids":map(lambda n:n[1], self.pids_) }
        self.sock.send(yaml.dump(order))
        #print "DEBUG;[OnUpdateCont] sended"
      except socket.error:
        self.sock.close()
        self.sock = None
        print "ERROR:[OnUpdateCont-02] cannot send to proc_manager"
        return
      # ftrace開始
      self.sbtn.SetBackgroundColour("#CCFFCC")
      self.update = True
      self.dtime = 0
      #print "DEBUG;[OnUpdateCont] cont on"
      self.timer.Start(100)

    else:
      # ftrace停止
      self.sbtn.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_BACKGROUND))
      self.update = False
      #print "DEBUG;[OnUpdateCont] cont off"
      self.timer.Stop()

      if self.sock is not None:
        self.sock.close()
        self.sock = None

  # MyFrame: cpu/procボタンによる CPU毎／ROSnode毎の表示形態変更処理関数
  def OnChangeView(self, event):
    self.view_ = 1 if self.view_ == 0 else 0
    self.ChangeView(True)

  # MyFrame: closeボタンによる終了処理関数
  def OnExit(self, event):
    sys.exit(0)

  # MyFrame: タイマ割り込み処理関数(ftraceデータ受信、描画)
  def OnTimer(self, event):
    dat = ""
    while True:
      (r, _, _) = select.select([self.sock], [], [], 0.01) # 10ms待ち
      if len(r) <= 0:
        break
      d = self.sock.recv(1000000)
      if len(d) <= 0:
        break
      dat += d

    if len(dat) > 0:
      #print "DEBUG:[OnTimer] rlen=%d" % len(dat)
      dat = pickle.loads(dat)
      self.UpdateGraph(dat)

  # MyFrame: グラフ描画更新処理関数
  def UpdateGraph(self, dat):
    # 新しいデータの最後と最初の時刻を取得
    etime = 0 # 最後の時刻(グラフの右端となる)
    for cpuno in dat:
      for d in dat[cpuno]:
        if d[1] > etime:
          etime = d[1]
    stime = etime - self.dtimespan # 最初の時刻(グラフの左端となる)
    ctime = datetime.datetime.now() # 現在時刻の取得

    # 過去描画のシフト
    if self.itmcount_ > 0:
      # シフト量の計算
      sft = self.cgtw_ * (etime - self.dtime) / self.dtimespan # シフト量 [pixel]
      if sft > self.cgtw_:
        sft = self.cgtw_ # 全領域クリア

      for n in range(0, self.itmcount_):
        dc = wx.PaintDC(self.cgpanels_[n])
        dc.SetPen(wx.Pen(self.bgcol_))
        dc.SetBrush(wx.Brush(self.bgcol_))
        # 過去データの描画をシフト
        dc.Blit(0, 0, self.cgtw_ - sft, self.cgth_, dc, sft, 0)
        # 新データの描画領域クリア
        dc.DrawRectangle(self.cgtw_ - sft, 0, sft, self.cgth_)
    self.dtime = etime

    # 新データのグラフ描画
    for cpuno in range(0, self.cpucount_):
      pid = 0 # 前回pid
      ptm = 0 # 前回時刻
      for d in dat[cpuno]:
        cid = d[0] # 今回pid
        ctm = d[1] # 今回時刻
        col = ''   # 描画色(前回のpidに依存)
        for (_, p) in self.pids_:
          if p == pid:
            col = self.GetColor(cpuno, pid)
            break
        if len(col) == 0:
          if self.view_ == 0 and pid != 0:
            col = "#cccccc"
          else:
            ptm = ctm
            pid = cid
            continue
        if self.view_ == 0:
          n = cpuno
        else:
          n = self.labels_.index(pid)
        dc = wx.PaintDC(self.cgpanels_[n])
        dc.SetPen(wx.Pen(col))
        dc.SetBrush(wx.Brush(col))
        w = int(self.cgtw_ * (ctm - ptm) / self.dtimespan + 1)
        p = int(self.cgtw_ * (ptm - stime) / self.dtimespan)
        if (p + w) > (self.cgtw_ - 1):
          p = self.cgtw_ - w - 1
        dc.DrawRectangle(p, 0, w, self.cgth_)
        ptm = ctm
        pid = cid

    # 時刻描画
    dc = wx.PaintDC(self.cgpanels_[self.itmcount_])
    dc.SetPen(wx.Pen(self.bgcol_))
    dc.SetBrush(wx.Brush(self.bgcol_))
    dc.DrawRectangle(0, 0, self.cgtw_, self.cgth_) # 時刻欄の描画領域クリア
    dc.SetTextForeground((0,0,0))
    tint = self.cgtw_ / self.dtimespan # 時刻表示間隔
    tofs = self.cgtw_ * (1 - (ctime.microsecond / self.dtimespan / 1000000)) # 時刻表示オフセット
    tmin = ctime.minute
    tsec = ctime.second
    for t in range(0, 4):
      ttext = "%02d:%02d" % (tmin, tsec)
      txtw,_ = dc.GetTextExtent(ttext) # 時刻表示の文字列幅の取得
      dc.DrawText(ttext, tofs - (txtw / 2), 0) # 時刻描画
      tofs -= tint
      tsec -= 1
      if tsec < 0:
        tsec = 59
        tmin -= 1
        if tmin < 0:
          tmin = 59

  # MyFrame: グラフ描画色の取得関数
  def GetColor(self, cpuno, pid):
    if self.view_ == 0:
      i = 0
      for (_, p) in self.pids_:
        if p == pid:
          return self.itmcolors_[i % len(self.itmcolors_)]
        i += 1
      return self.itmcolors_[0]
    else:
      return self.itmcolors_[cpuno % len(self.itmcolors_)]

  # MyFrame: ROSノードのpidを取得する関数
  def getRosNodes(self):
    nodes = []
    try:
      nodenames = rosnode.get_node_names(None)
    except Exception as inst:
      print "ERROR:[getRosNodes-01] ", inst
      return nodes

    for nodename in nodenames:
      #rosnode.rosnode_info(nodename)
      api = rosnode.get_api_uri(rosgraph.Master("/rosnode"), nodename)
      if api:
        try:
          node = ServerProxy(api)
          code, msg, pid = node.getPid("/rosnode")
          if code == 1:
            res = re.search("^(.*)_[0-9]+$", nodename)
            while res is not None:
              nodename = res.group(1)
              res = re.search("^(.*)_[0-9]+$", nodename)
            nodes.append((nodename, pid))
        except:
          pass
    return nodes

# メイン処理
if __name__ == "__main__":
  myapp = wx.App(0)
  frame = MyFrame(None, -1, "Ftrace")
  frame.Show()
  myapp.MainLoop()
