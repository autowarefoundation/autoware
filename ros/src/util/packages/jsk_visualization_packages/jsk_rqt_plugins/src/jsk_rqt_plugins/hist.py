#!/usr/bin/env python
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QIcon, QMenu, QWidget
from python_qt_binding.QtGui import QWidget, QVBoxLayout, QSizePolicy, QColor
from rqt_py_common.topic_completer import TopicCompleter
from matplotlib.colors import colorConverter
from rqt_py_common.topic_helpers import is_slot_numeric
from rqt_plot.rosplot import ROSData as _ROSData
from rqt_plot.rosplot import RosPlotException
from matplotlib.collections import (PolyCollection, 
                                    PathCollection, LineCollection)
import matplotlib
import matplotlib.patches as mpatches
import rospkg
import rospy
from cStringIO import StringIO
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jsk_recognition_msgs.msg import HistogramWithRange, HistogramWithRangeBin

import os, sys
import argparse

try:
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    # work around bug in dateutil
    import sys
    import thread
    sys.modules['_thread'] = thread
    from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QTAgg as NavigationToolbar
from matplotlib.figure import Figure

import numpy as np
import matplotlib.pyplot as plt

class ROSData(_ROSData):
    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                return val
            for f in self.field_evals:
                val = f(val)
            return val
        except IndexError:
            self.error = RosPlotException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosPlotException("[%s] value was not numeric: %s" % (self.name, val))



class HistogramPlot(Plugin):
    def __init__(self, context):
        super(HistogramPlot, self).__init__(context)
        self.setObjectName('HistogramPlot')
        self._args = self._parse_args(context.argv())
        self._widget = HistogramPlotWidget(self._args.topics)
        context.add_widget(self._widget)
    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_histogram_plot', add_help=False)
        HistogramPlot.add_arguments(parser)
        args = parser.parse_args(argv)
        return args
    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_histogram plugin')
        group.add_argument('topics', nargs='?', default=[], help='Topics to plot')
        
class HistogramPlotWidget(QWidget):
    _redraw_interval = 40
    def __init__(self, topics):
        super(HistogramPlotWidget, self).__init__()
        self.setObjectName('HistogramPlotWidget')
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('jsk_rqt_plugins'), 
                               'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.cv_bridge = CvBridge()
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatHistogramPlot(self)
        self.data_plot_layout.addWidget(self.data_plot)
        self._topic_completer = TopicCompleter(self.topic_edit)
        self._topic_completer.update_topics()
        self.topic_edit.setCompleter(self._topic_completer)
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent
        self._start_time = rospy.get_time()
        self._rosdata = None
        if len(topics) != 0:
            self.subscribe_topic(topics)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._update_plot_timer.start(self._redraw_interval)
    @Slot('QDropEvent*')
    def dropEvent(self, event):
        if event.mimeData().hasText():
            topic_name = str(event.mimeData().text())
        else:
            droped_item = event.source().selectedItems()[0]
            topic_name = str(droped_item.data(0, Qt.UserRole))
        self.subscribe_topic(topic_name)
    @Slot()
    def on_topic_edit_returnPressed(self):
        if self.subscribe_topic_button.isEnabled():
            self.subscribe_topic(str(self.topic_edit.text()))
    @Slot()
    def on_subscribe_topic_button_clicked(self):
        self.subscribe_topic(str(self.topic_edit.text()))

    def subscribe_topic(self, topic_name):
        self.topic_with_field_name = topic_name
        self.pub_image = rospy.Publisher(topic_name + "/histogram_image", Image)
        if not self._rosdata:
            self._rosdata = ROSData(topic_name, self._start_time)
        else:
            if self._rosdata != topic_name:
                self._rosdata.close()
                self.data_plot.clear()
                self._rosdata = ROSData(topic_name, self._start_time)
            else:
                rospy.logwarn("%s is already subscribed", topic_name)
        
    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()
    @Slot()
    def on_clear_button_clicked(self):
        self.data_plot.clear()
    
    @Slot(bool)
    def on_pause_button_clicked(self, checked):
        self.enable_timer(not checked)
    
    def update_plot(self):
        if not self._rosdata:
            return
        data_x, data_y = self._rosdata.next()

        if len(data_y) == 0:
            return
        axes = self.data_plot._canvas.axes
        axes.cla()
        if self._rosdata.sub.data_class is HistogramWithRange:
            xs = [y.count for y in data_y[-1].bins]
            pos = [y.min_value for y in data_y[-1].bins]
            widths = [y.max_value - y.min_value for y in data_y[-1].bins]
            axes.set_xlim(xmin=pos[0], xmax=pos[-1] + widths[-1])
        else:
            xs = data_y[-1]
            pos = np.arange(len(xs))
            widths = [1] * len(xs)
            axes.set_xlim(xmin=0, xmax=len(xs))
        #axes.xticks(range(5))
        for p, x, w in zip(pos, xs, widths):
            axes.bar(p, x, color='r', align='center', width=w)
        axes.legend([self.topic_with_field_name], prop={'size': '8'})
        self.data_plot._canvas.draw()
        buffer = StringIO()
        self.data_plot._canvas.figure.savefig(buffer, format="png")
        buffer.seek(0)
        img_array = np.asarray(bytearray(buffer.read()), dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_COLOR)
        self.pub_image.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
class MatHistogramPlot(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None):
            super(MatHistogramPlot.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()
        def resizeEvent(self, event):
            super(MatHistogramPlot.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()
    def __init__(self, parent=None):
        super(MatHistogramPlot, self).__init__(parent)
        self._canvas = MatHistogramPlot.Canvas()
        self._toolbar = NavigationToolbar(self._canvas, self._canvas)
        vbox = QVBoxLayout()
        vbox.addWidget(self._toolbar)
        vbox.addWidget(self._canvas)
        self.setLayout(vbox)
    def redraw(self):
        pass
    def clear(self):
        self._canvas.axes.cla()
        self._canvas.draw()
