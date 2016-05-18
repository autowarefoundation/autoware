#!/usr/bin/env python
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QWidget,
                                     QPainter, QColor, QFont, QBrush, 
                                     QPen)
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math

class DRCEnvironmentViewer(Plugin):
    def __init__(self, context):
        super(DRCEnvironmentViewer, self).__init__(context)
        self.setObjectName("DRCEnvironmentViewer")
        self._widget = DRCEnvironmentViewerWidget()
        context.add_widget(self._widget)
        
class DRCEnvironmentViewerWidget(QWidget):
    _SMILEY = ":)"
    _FROWN = ":("
    _OK_COLOR = QColor("#18FFFF")
    _DISABLED_COLOR = QColor("#BDBDBD")
    _BLACKOUT_COLOR = QColor("#F44336")
    def __init__(self):
        self.lock = Lock()
        super(DRCEnvironmentViewerWidget, self).__init__()
        self.is_disabled = False
        self.is_blackout = False
        self.next_whiteout_time = rospy.Time.now()
        self.blackout_time = rospy.Time.now()
        self.event = None
        self.sub_is_disabled = rospy.Subscriber("/drc_2015_environment/is_disabled",
                                                Bool, self.isDisabledCallback)
        self.sub_is_blackout = rospy.Subscriber("/drc_2015_environment/is_blackout",
                                                Bool, self.isBlackoutCallback)
        self.sub_next_whiteout_time = rospy.Subscriber("/drc_2015_environment/next_whiteout_time",
                                                       Time, self.nextWhiteoutTimeCallback)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(1000 / 15)
    def isDisabledCallback(self, msg):
        with self.lock:
            self.is_disabled = msg.data
    def isBlackoutCallback(self, msg):
        with self.lock:
            if not self.is_blackout and msg.data:
                self.blackout_time = rospy.Time.now()
            self.is_blackout = msg.data
    def nextWhiteoutTimeCallback(self, msg):
        with self.lock:
            self.next_whiteout_time = msg.data
    def redraw(self):
        self.update()
        # if self.event:
        #     self.paintEvent(self.event)
    def paintEvent(self, event):
        with self.lock:
            self.event = event
            rect = event.rect()
            qp = QPainter()
            qp.begin(self)
            radius = min(rect.width(), rect.height()) - 50
            qp.setFont(QFont('Helvetica', 100))
            qp.setPen(QPen(QBrush(QColor(255, 255, 255)), 20))
            
            if self.is_disabled:
                qp.fillRect(rect, self._DISABLED_COLOR)
                qp.drawText(rect, QtCore.Qt.AlignCenter, self._FROWN)
            elif self.is_blackout:
                qp.fillRect(rect, self._BLACKOUT_COLOR)
                qp.drawText(rect, QtCore.Qt.AlignCenter, self._FROWN)
                time_diff = (self.next_whiteout_time - rospy.Time.now()).to_sec()
                if time_diff < 0:
                    time_diff = 0
                time_ratio = time_diff / (self.next_whiteout_time - self.blackout_time).to_sec()
                qp.setFont(QFont('Helvetica', 30))
                qp.drawText(0, rect.height() - 150, rect.width(), 150, QtCore.Qt.AlignCenter, "%.1f sec" % time_diff)
                # 0-360
                if time_ratio > 0:
                    rad = int(math.fmod(time_ratio * 360 + 90*16, 360) * 16)
                    qp.drawArc((rect.width() - radius) / 2, (rect.height() - radius) / 2, radius, radius, 90*16, rad)
            else:
                qp.fillRect(rect, self._OK_COLOR)
                qp.drawText(rect, QtCore.Qt.AlignCenter, self._SMILEY)
            qp.end()
