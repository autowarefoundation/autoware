from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QWidget,
                                     QPainter, QColor, QFont, QBrush, 
                                     QPen, QMessageBox, QSizePolicy,
                                     QLabel, QComboBox)
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent, QSize
from threading import Lock
import rospy
import roslib
import rqt_plot
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys

from std_msgs.msg import String

from .util import get_slot_type_field_names
from .hist import ROSData
from .button_general import LineEditDialog


class StringLabel(Plugin):
    """
    rqt plugin to provide simple label
    """
    def __init__(self, context):
        super(StringLabel, self).__init__(context)
        self.setObjectName("StringLabel")
        self._widget = StringLabelWidget()
        context.add_widget(self._widget)
    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(plugin_settings, instance_settings)
    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(plugin_settings, instance_settings)
    def trigger_configuration(self):
        self._widget.trigger_configuration()


class StringLabelWidget(QWidget):
    def __init__(self):
        super(StringLabelWidget, self).__init__()
        self.lock = Lock()
        vbox = QtGui.QVBoxLayout(self)
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignLeft)
        self.label.setSizePolicy(QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored))
        font = QFont("Helvetica", 14)
        self.label.setFont(font)
        self.label.setWordWrap(True)
        vbox.addWidget(self.label)
        self.string_sub = None
        self._string_topics = []
        self._update_topic_timer = QTimer(self)
        self._update_topic_timer.timeout.connect(self.updateTopics)
        self._update_topic_timer.start(1000)
        self._active_topic = None
        # to update label visualization
        self._dialog = LineEditDialog()
        self._rosdata = None
        self._start_time = rospy.get_time()
        self._update_label_timer = QTimer(self)
        self._update_label_timer.timeout.connect(self.updateLabel)
        self._update_label_timer.start(40)
    def trigger_configuration(self):
        self._dialog.exec_()
        self.setupSubscriber(self._dialog.value)
    def updateLabel(self):
        if not self._rosdata:
            return
        try:
            _, data_y = self._rosdata.next()
        except rqt_plot.rosplot.RosPlotException, e:
            self._rosdata = None
            return
        if len(data_y) == 0:
            return
        latest = data_y[-1]  # get latest data
        # supports std_msgs/String as well as string data nested in rosmsg
        if type(latest) == String:
            self.string = latest.data
        else:
            self.string = latest
        try:
            self.label.setText(self.string)
        except TypeError, e:
            rospy.logwarn(e)
    def updateTopics(self):
        need_to_update = False
        for (topic, topic_type) in rospy.get_published_topics():
            msg = roslib.message.get_message_class(topic_type)
            field_names = get_slot_type_field_names(msg, slot_type='string')
            for field in field_names:
                string_topic = topic + field
                if string_topic not in self._string_topics:
                    self._string_topics.append(string_topic)
                    need_to_update = True
        if need_to_update:
            self._string_topics = sorted(self._string_topics)
            self._dialog.combo_box.clear()
            for topic in self._string_topics:
                self._dialog.combo_box.addItem(topic)
            if self._active_topic:
                if self._active_topic not in self._string_topics:
                    self._string_topics.append(self._active_topic)
                    self._dialog.combo_box.addItem(self._active_topic)
                self._dialog.combo_box.setCurrentIndex(self._string_topics.index(self._active_topic))
    def setupSubscriber(self, topic):
        if not self._rosdata:
            self._rosdata = ROSData(topic, self._start_time)
        else:
            if self._rosdata != topic:
                self._rosdata.close()
                self._rosdata = ROSData(topic, self._start_time)
            else:
                rospy.logwarn("%s is already subscribed", topic)
        self._active_topic = topic
    def onActivated(self, number):
        self.setupSubscriber(self._string_topics[number])
    def save_settings(self, plugin_settings, instance_settings):
        if self._active_topic:
            instance_settings.set_value("active_topic", self._active_topic)
    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("active_topic"):
            topic = instance_settings.value("active_topic")
            self._dialog.combo_box.addItem(topic)
            self.setupSubscriber(topic)

    









