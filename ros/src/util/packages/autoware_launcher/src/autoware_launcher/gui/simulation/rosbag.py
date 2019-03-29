from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.gui  import widgets



class AwRosbagSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwRosbagSimulatorWidget, self).__init__()
        self.rosbag_mode_proc = QtCore.QProcess(self)
        self.rosbag_info_proc = QtCore.QProcess(self)
        self.rosbag_play_proc = QtCore.QProcess(self)

        self.rosbag_file   = widgets.AwFileSelect(self)
        self.rosbag_info   = QtWidgets.QPushButton("Info")
        self.rosbag_text   = QtWidgets.QLabel("No information")
        self.rosbag_enable = QtWidgets.QCheckBox()
        self.rosbag_label  = QtWidgets.QLabel("Simulation Mode")
        self.rosbag_play   = QtWidgets.QPushButton("Play")
        self.rosbag_stop   = QtWidgets.QPushButton("Stop")
        self.rosbag_pause  = QtWidgets.QPushButton("Pause")
        self.rosbag_state  = QtWidgets.QLabel()
        #self.rosbag_stime  = QtWidgets.QLineEdit()
        #start time
        #repeat
        #rate

        self.rosbag_enable.stateChanged.connect(self.simulation_mode_changed)
        self.rosbag_info.clicked.connect(self.rosbag_info_requested)
        self.rosbag_info_proc.finished.connect(self.rosbag_info_completed)

        self.rosbag_play.clicked.connect(self.rosbag_started)
        self.rosbag_stop.clicked.connect(self.rosbag_stopped)
        self.rosbag_play_proc.finished.connect(self.rosbag_finished)
        self.rosbag_play_proc.readyReadStandardOutput.connect(self.rosbag_output)

        self.rosbag_pause.setCheckable(True)
        self.rosbag_pause.toggled.connect(self.rosbag_paused)

        self.setStyleSheet("QCheckBox::indicator { width: 28px; height: 28px; }")
        self.rosbag_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.rosbag_text.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.rosbag_enable,      0, 0)
        layout.addWidget(self.rosbag_label,       0, 1)
        layout.addWidget(self.rosbag_play,        0, 2)
        layout.addWidget(self.rosbag_stop,        0, 3)
        layout.addWidget(self.rosbag_pause,       0, 4)
        layout.addWidget(self.rosbag_state,       1, 0, 1, 5)
        layout.addWidget(self.rosbag_file.path,   2, 0, 1, 3)
        layout.addWidget(self.rosbag_file.button, 2, 3)
        layout.addWidget(self.rosbag_info,        2, 4)
        layout.addWidget(self.rosbag_text,        3, 0, 1, 5)
        self.setLayout(layout)
        self.simulation_mode_disabled()

    def simulation_mode_changed(self, state):
        if state == QtCore.Qt.Checked:   self.simulation_mode_enabled()
        if state == QtCore.Qt.Unchecked: self.simulation_mode_disabled()

    def simulation_mode_enabled(self):
        self.rosbag_mode_proc.start("rosparam set /use_sim_time true")
        self.rosbag_stopped()

    def simulation_mode_disabled(self):
        self.rosbag_mode_proc.start("rosparam set /use_sim_time false")
        self.rosbag_stopped()
        self.rosbag_play.setEnabled(False)

    def rosbag_info_requested(self):
        self.rosbag_info_proc.start("rosbag info " + self.rosbag_file.path.text())

    def rosbag_info_completed(self):
        stdout = self.rosbag_info_proc.readAllStandardOutput().data().decode('utf-8')
        stderr = self.rosbag_info_proc.readAllStandardError().data().decode('utf-8')
        self.rosbag_text.setText(stdout + stderr)

    def rosbag_started(self):
        xml = myutils.package("resources/rosbagplay.xml")
        arg = self.rosbag_file.path.text()
        self.rosbag_play_proc.start('roslaunch {} options:="{}" bagfile:={}'.format(xml, "--clock --start=0", arg))
        self.rosbag_play_proc.processId()
        self.rosbag_play.setEnabled(False)
        self.rosbag_stop.setEnabled(True)
        self.rosbag_pause.setEnabled(True)

    def rosbag_stopped(self):
        self.rosbag_play_proc.terminate()
        self.rosbag_finished()
    
    def rosbag_finished(self):
        self.rosbag_play.setEnabled(True)
        self.rosbag_stop.setEnabled(False)
        self.rosbag_pause.setEnabled(False)
        self.rosbag_pause.setChecked(False)

    def rosbag_paused(self, checked):
        self.rosbag_play_proc.write(" ")

    def rosbag_output(self):
        #print(self.rosbag_play_proc.readAllStandardOutput().data().decode("utf-8"))
        stdout = str(self.rosbag_play_proc.readAllStandardOutput()).split("\r")
        if 2 <= len(stdout):
            self.rosbag_state.setText(stdout[-2])

