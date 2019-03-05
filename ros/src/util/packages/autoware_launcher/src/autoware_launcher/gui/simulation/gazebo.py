from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
from autoware_launcher.core import myutils


class AwGazeboSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwGazeboSimulatorWidget, self).__init__()
        self.gazebo_process = QtCore.QProcess(self)

        self.launch_button = QtWidgets.QPushButton("Launch")
        self.setup_button  = QtWidgets.QPushButton("Initial Setup")
        self.world_buttons = []
        self.world_buttons.append(self.__create_radio_button("simple", "simple"))
        self.world_buttons.append(self.__create_radio_button("mcity", "mcity"))
        self.world_buttons.append(self.__create_radio_button("city sim", "citysim_gazebo7"))
        self.use_gpu_box = QtWidgets.QCheckBox("Use GPU")

        self.world_buttons[0].setChecked(True)
        self.setup_button.clicked.connect(self.__exec_setup_script)
        self.launch_button.setCheckable(True)
        self.launch_button.toggled.connect(self.__exec_simulator)
        self.gazebo_process.finished.connect(self.__simulator_finished)

        world_group = QtWidgets.QGroupBox("World")
        world_group.setLayout(QtWidgets.QVBoxLayout())
        for world_button in self.world_buttons:
            world_group.layout().addWidget(world_button)
        world_group.layout().addStretch()

        config_group = QtWidgets.QGroupBox("Config")
        config_group.setLayout(QtWidgets.QVBoxLayout())
        config_group.layout().addWidget(self.use_gpu_box)
        config_group.layout().addStretch()

        hlayout1 = QtWidgets.QHBoxLayout()
        hlayout1.addStretch()
        hlayout1.addWidget(self.setup_button)

        hlayout2 = QtWidgets.QHBoxLayout()
        hlayout2.addWidget(world_group)
        hlayout2.addWidget(config_group)

        self.setLayout(QtWidgets.QVBoxLayout())
        self.layout().addLayout(hlayout1)
        self.layout().addLayout(hlayout2)
        self.layout().addWidget(self.launch_button)

    def __exec_setup_script(self):
        import subprocess
        subprocess.call(["gnome-terminal", "-e", "rosrun vehicle_gazebo_simulation_launcher setup.sh"])

    def __exec_simulator(self, checked):
        if checked:
            use_gpu = self.use_gpu_box.isChecked()
            for world_button in self.world_buttons:
                if world_button.isChecked():
                    world = world_button.parameter
            command = ["roslaunch", "vehicle_gazebo_simulation_launcher", "gazebo_launcher.launch"]
            command.append("world_name:={}".format(world))
            command.append("gpu:={}".format("true" if use_gpu else "false"))
            self.gazebo_process.start(" ".join(command))
        else:
            self.launch_button.setEnabled(False)
            self.gazebo_process.terminate()

    def __simulator_finished(self):
            self.launch_button.setEnabled(True)

    def __create_radio_button(self, text, parameter):
        radio = QtWidgets.QRadioButton(text)
        radio.parameter = parameter
        return radio
