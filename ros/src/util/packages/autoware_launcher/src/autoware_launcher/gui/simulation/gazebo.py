from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from autoware_launcher.core import myutils



class AwGazeboSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwGazeboSimulatorWidget, self).__init__()