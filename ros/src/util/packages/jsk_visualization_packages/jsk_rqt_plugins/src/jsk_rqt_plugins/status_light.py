from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QWidget, 
                                     QPainter, QColor, QFont, QBrush, 
                                     QPen, QMessageBox, QSizePolicy, 
                                     QLabel, QComboBox)
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent, QSize
from threading import Lock
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys

from std_msgs.msg import UInt8
from image_view2_wrapper import ComboBoxDialog

class StatusLight(Plugin):
    """
    rqt plugin to show light like ultra-man's light.
    It subscribes std_msgs/UInt8 topic and the value means:
    0 == Unknown (gray)
    1 == Success (green)
    2 == Warn    (yellow)
    """
    def __init__(self, context):
        super(StatusLight, self).__init__(context)
        self.setObjectName("StatusLight")
        self._widget = StatusLightWidget()
        context.add_widget(self._widget)
    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)
    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
    def trigger_configuration(self):
        self._widget.trigger_configuration()

class StatusLightWidget(QWidget):
    _UNKNOWN_COLOR = QColor("#dddddd")
    _SUCCESS_COLOR = QColor("#18FFFF")
    _WARN_COLOR = QColor("#FFCA00")
    _ERROR_COLOR = QColor("#F44336")
    def __init__(self):
        super(StatusLightWidget, self).__init__()
        self.lock = Lock()
        self.status_sub = None
        self.status = 0
        self._status_topics = []
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)
        self._active_topic = None
        self._dialog = ComboBoxDialog()
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.redraw)
        self._update_plot_timer.start(1000 / 15)
        
    def redraw(self):
        self.update()
    def paintEvent(self, event):
        with self.lock:
            if self.status == 1:
                color = self._SUCCESS_COLOR
            elif self.status == 2:
                color = self._WARN_COLOR
            else:
                color = self._UNKNOWN_COLOR
            rect = event.rect()
            qp = QPainter()
            qp.begin(self)
            radius = min(rect.width(), rect.height()) - 100
            qp.setFont(QFont('Helvetica', 100))
            qp.setPen(QPen(QBrush(color), 50))
            qp.setBrush(color)
            qp.drawEllipse((rect.width() - radius) / 2, (rect.height() - radius) / 2, 
                           radius, radius)
            qp.end()
            return

    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._status_topics[self._dialog.number])
    def updateTopics(self):
        need_to_update = False
        for (topic, topic_type) in rospy.get_published_topics():
            if topic_type == "std_msgs/UInt8":
                if not topic in self._status_topics:
                    self._status_topics.append(topic)
                    need_to_update = True
        if need_to_update:
            self._status_topics = sorted(self._status_topics)
            self._dialog.combo_box.clear()
            for topic in self._status_topics:
                self._dialog.combo_box.addItem(topic)
            if self._active_topic:
                if self._active_topic not in self._status_topics:
                    self._status_topics.append(self._active_topic)
                    self._dialog.combo_box.addItem(self._active_topic)
                self._dialog.combo_box.setCurrentIndex(self._status_topics.index(self._active_topic))
    def setupSubscriber(self, topic):
        if self.status_sub:
            self.status_sub.unregister()
        self.status_sub = rospy.Subscriber(topic, UInt8,
                                           self.statusCallback)
        self._active_topic = topic
    def onActivated(self, number):
        self.setupSubscriber(self._status_topics[number])
    def statusCallback(self, msg):
        self.status = msg.data
    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value("active_topic", self._active_topic)
    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("active_topic"):
            topic = instance_settings.value("active_topic")
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)
