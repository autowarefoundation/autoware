#import xml.etree.ElementTree as xmltree

import rospkg
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

def main(sys_argv):


    application = QtWidgets.QApplication(sys_argv)


    rospack = rospkg.RosPack()
    combo = QtWidgets.QComboBox()
    for package in sorted(rospack.list()):
        combo.addItem(package)
    combo.setEditable(True)
    combo.setCurrentIndex(combo.findText("autoware_launcher"))

    widget = QtWidgets.QWidget()
    widget.setLayout(QtWidgets.QHBoxLayout())
    widget.layout().addWidget(QtWidgets.QLabel("Package"))
    widget.layout().addWidget(combo)
    widget.layout().addWidget(QtWidgets.QLabel("File"))
    widget.layout().addWidget(QtWidgets.QLineEdit())
    widget.layout().addWidget(QtWidgets.QPushButton())
    widget.show()
    return application.exec_()
