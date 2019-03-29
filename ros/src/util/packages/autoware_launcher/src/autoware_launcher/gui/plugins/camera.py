from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.gui  import widgets



def plugin_widgets():
    return {}


"""
class AwCameraCalibFrame(AwFileSelect):

    def __init__(self, guimgr, node, opts):
        super(AwCameraCalibFrame, self).__init__(guimgr, node, opts)

        calib = QtWidgets.QPushButton("Calib")
        calib.setCheckable(True)
        calib.toggled.connect(self.calibrate)
        self.add_button(calib)

    def refresh_image_topic(self):
        from subprocess import Popen, PIPE
        command = "rostopic find sensor_msgs/Image"
        process = Popen(command.split(), stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        self.topic_name.clear()
        for topic in stdout.split("\n"):
            if topic: self.topic_name.addItem(topic)

    def calibrate(self, checked):

        self.intrinsic_calibrator = QtCore.QProcess(self)
        self.extrinsic_calibrator = QtCore.QProcess(self)
        self.corner_size_x    = QtWidgets.QLineEdit("8")
        self.corner_size_y    = QtWidgets.QLineEdit("6")
        self.square_length    = QtWidgets.QLineEdit("0.1")
        self.topic_name       = QtWidgets.QComboBox()
        self.topic_refresh    = QtWidgets.QPushButton("Refresh")
        self.intrinsic_file   = QtWidgets.QLineEdit()
        self.intrinsic_browse = QtWidgets.QPushButton("Browse")
        self.calib_intrinsic  = QtWidgets.QPushButton("Calibrate Intrinsic")
        self.calib_extrinsic  = QtWidgets.QPushButton("Calibrate Extrinsic")

        self.refresh_image_topic()
        self.topic_refresh.clicked.connect(self.refresh_image_topic)
        self.intrinsic_browse.clicked.connect(self.select_intrinsic)

        self.calib_intrinsic.setCheckable(True)
        self.calib_intrinsic.toggled.connect(self.calibrate_intrinsic)
        self.calib_extrinsic.setCheckable(True)
        self.calib_extrinsic.toggled.connect(self.calibrate_extrinsic)

        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QGridLayout())
        widget.layout().addWidget(QtWidgets.QLabel("Square Corners"), 0, 0)
        widget.layout().addWidget(self.corner_size_x,                 0, 1)
        widget.layout().addWidget(QtWidgets.QLabel("x"),              0, 2)
        widget.layout().addWidget(self.corner_size_y,                 0, 3)
        widget.layout().addWidget(QtWidgets.QLabel("Square Length"),  0, 4)
        widget.layout().addWidget(self.square_length,                 0, 5)
        widget.layout().addWidget(QtWidgets.QLabel("Image Topic"),    1, 0)
        widget.layout().addWidget(self.topic_name,                    1, 1, 1, 4)
        widget.layout().addWidget(self.topic_refresh,                 1, 5)
        widget.layout().addWidget(QtWidgets.QLabel("Intrinsic File"), 2, 0)
        widget.layout().addWidget(self.intrinsic_file,                2, 1, 1, 4)
        widget.layout().addWidget(self.intrinsic_browse,              2, 5)
        widget.layout().addWidget(self.calib_intrinsic,               3, 0)
        widget.layout().addWidget(self.calib_extrinsic,               3, 1)

        window = QtWidgets.QMainWindow(self)
        window.setCentralWidget(widget)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.show()

    def select_intrinsic(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
        if filename:
            self.intrinsic_file.setText(filename)

    def calibrate_intrinsic(self, checked):
        if checked:
            command = "rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square {} --size {}x{} image:={}"
            command = command.format(self.square_length.text(), self.corner_size_x.text(), self.corner_size_y.text(), self.topic_name.currentText())
            self.intrinsic_calibrator.start(command )
        else:
            self.intrinsic_calibrator.terminate()

    def calibrate_extrinsic(self, checked):
        if checked:
            command = "roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:={} image_src:={}"
            command = command.format(self.intrinsic_file.text(), self.topic_name.currentText())
            self.extrinsic_calibrator.start(command )
        else:
            self.extrinsic_calibrator.terminate()
"""