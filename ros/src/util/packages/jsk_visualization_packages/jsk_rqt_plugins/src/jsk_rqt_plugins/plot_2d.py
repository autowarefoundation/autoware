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
from jsk_recognition_msgs.msg import PlotData
import numpy as np
#from sklearn import linear_model, datasets
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



class Plot2D(Plugin):
    def __init__(self, context):
        super(Plot2D, self).__init__(context)
        self.setObjectName('Plot2D')
        self._args = self._parse_args(context.argv())
        self._widget = Plot2DWidget(self._args.topics)
        self._widget.is_line = self._args.line
        self._widget.fit_line = self._args.fit_line
        self._widget.fit_line_ransac = self._args.fit_line_ransac
        self._widget.fit_line_ransac_outlier = self._args.fit_line_ransac_outlier
        self._widget.xtitle = self._args.xtitle
        self._widget.ytitle = self._args.ytitle
        self._widget.no_legend = self._args.no_legend
        self._widget.sort_x = self._args.sort_x
        context.add_widget(self._widget)
    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_histogram_plot', add_help=False)
        Plot2D.add_arguments(parser)
        args = parser.parse_args(argv)
        return args
    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_histogram plugin')
        group.add_argument('topics', nargs='?', default=[], help='Topics to plot')
        group.add_argument('--line', action="store_true", help="Plot with lines instead of scatter")
        group.add_argument('--fit-line', action="store_true", help="Plot line with least-square fitting")
        group.add_argument('--fit-line-ransac', action="store_true", help="Plot line with RANSAC")
        group.add_argument('--fit-line-ransac-outlier', type=float, default=0.1, help="Plot line with RANSAC")
        group.add_argument('--xtitle', help="Title in X axis")
        group.add_argument('--ytitle', help="Title in Y axis")
        group.add_argument('--no-legend', action="store_true")
        group.add_argument('--sort-x', action="store_true")
class Plot2DWidget(QWidget):
    _redraw_interval = 40
    def __init__(self, topics):
        super(Plot2DWidget, self).__init__()
        self.setObjectName('Plot2DWidget')
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('jsk_rqt_plugins'), 
                               'resource', 'plot_histogram.ui')
        loadUi(ui_file, self)
        self.cv_bridge = CvBridge()
        self.subscribe_topic_button.setIcon(QIcon.fromTheme('add'))
        self.pause_button.setIcon(QIcon.fromTheme('media-playback-pause'))
        self.clear_button.setIcon(QIcon.fromTheme('edit-clear'))
        self.data_plot = MatPlot2D(self)
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
        # matplotlib
        # concatenate x and y in order to sort
        concatenated_data = zip(data_y[-1].xs, data_y[-1].ys)
        if self.sort_x:
            concatenated_data.sort(key=lambda x: x[0])
        xs = [d[0] for d in concatenated_data]
        ys = [d[1] for d in concatenated_data]
        if self.is_line:
            axes.plot(xs, ys, label=self.topic_with_field_name)
        else:
            axes.scatter(xs, ys)
        # set limit
        axes.set_xlim(min(xs), max(xs))
        axes.set_ylim(min(ys), max(ys))
        # line fitting
        if self.fit_line:
            X = np.array(data_y[-1].xs)
            Y = np.array(data_y[-1].ys)
            A = np.array([X,np.ones(len(X))])
            A = A.T
            a,b = np.linalg.lstsq(A,Y)[0]
            axes.plot(X,(a*X+b),"g--", label="{0} x + {1}".format(a, b))
        if self.fit_line_ransac:
            #model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression(), min_samples=2,
                                                        #residual_threshold=self.fit_line_ransac_outlier)
            X = np.array(data_y[-1].xs).reshape((len(data_y[-1].xs), 1))
            Y = np.array(data_y[-1].ys)
            model_ransac.fit(X, Y)
            line_X = X
            line_y_ransac = model_ransac.predict(line_X)
            axes.plot(line_X, line_y_ransac, "r--",
                      label="{0} x + {1}".format(model_ransac.estimator_.coef_[0][0],
                                                 model_ransac.estimator_.intercept_[0]))
        if not self.no_legend:
            axes.legend(prop={'size': '8'})
        axes.grid()
        if self.xtitle:
            axes.set_xlabel(self.xtitle)
        if self.ytitle:
            axes.set_ylabel(self.ytitle)
        self.data_plot._canvas.draw()
        buffer = StringIO()
        self.data_plot._canvas.figure.savefig(buffer, format="png")
        buffer.seek(0)
        img_array = np.asarray(bytearray(buffer.read()), dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.CV_LOAD_IMAGE_COLOR)
        self.pub_image.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        
class MatPlot2D(QWidget):
    class Canvas(FigureCanvas):
        def __init__(self, parent=None):
            super(MatPlot2D.Canvas, self).__init__(Figure())
            self.axes = self.figure.add_subplot(111)
            self.figure.tight_layout()
            self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.updateGeometry()
        def resizeEvent(self, event):
            super(MatPlot2D.Canvas, self).resizeEvent(event)
            self.figure.tight_layout()
    def __init__(self, parent=None):
        super(MatPlot2D, self).__init__(parent)
        self._canvas = MatPlot2D.Canvas()
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
