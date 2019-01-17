import os
import rospy
import rospkg
import requests
import subprocess
import json

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget, QFileDialog
from python_qt_binding.QtGui import QPalette,QColor
from python_qt_binding.QtCore import QSettings

class RqtLgsvlSimulatorConfiguratorPlugin(Plugin):

    def __init__(self, context):
        super(RqtLgsvlSimulatorConfiguratorPlugin, self).__init__(context)
        rospack = rospkg.RosPack()
        self.settings = QSettings(rospack.get_path('lgsvl_simulator_bridge')+"/config/setting.dat",QSettings.IniFormat)
        # Give QObjects reasonable names
        self.setObjectName('RqtLgsvlSimulatorConfiguratorPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('lgsvl_simulator_bridge'), 'resource', 'LgsvlSimulatorConfigratorPlugin.ui')
        # Extend the widget with all atrributes and children from UI File
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('RqtLgsvlSimulatorConfiguratorPluginUi')
        # Show _widget.windowTitle on left-top of each plugin(when it's set in _widget).
        # This is useful when you open multiple plugins aat once. Also if you open multiple
        # instances of your plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' %d' % context.serial_number()))
        # Add widget to the user interface
        self.settings.beginGroup("simulator_params")
        context.add_widget(self._widget)
        ip = self.settings.value('ip')
        if ip != None:
            self._widget.ip_box.setText(ip)
        port = self.settings.value('port')
        if port != None:
            self._widget.port_box.setText(port)
        config_path = self.settings.value('config_path')
        if config_path != None:
            self._widget.configpath_box.setText(config_path)
        self.settings.endGroup()
        self.json_dict = {}
        self.instance_id = ""
        self.is_remote = False
        self._widget.button_config_ref.clicked[bool].connect(self._handle_config_ref_button_clicked)
        self._widget.button_config.clicked[bool].connect(self._handle_config_button_clicked)
        self._widget.launch_button.clicked[bool].connect(self._handle_launch_button_clicked)
        self._widget.terminate_button.clicked[bool].connect(self._handle_terminate_button_clicked)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.get_value(k, v)
        pass

    def restore_settings(self, pluign_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def _handle_launch_button_clicked(self):
        if str(self._widget.ip_box.text()) == "localhost" or str(self._widget.ip_box.text()) == "0.0.0.0" or str(self._widget.ip_box.text()) == "127.0.0.1":
            cmd = ["roslaunch","lgsvl_simulator_bridge","lgsvl_simulator.launch"]
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            self._widget.launch_button.setStyleSheet("background-color: #8fb8e0")
            self._widget.terminate_button.setStyleSheet("background-color: #FFFFFF")
            self.is_remote = False
            return
        else:
            cmd = ["roslaunch","lgsvl_simulator_bridge","bridge.launch"]
            self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)            
        address = "http://" + self._widget.ip_box.text() + ":" + self._widget.port_box.text()+"/simulator/launch"
        self.settings.beginGroup("simulator_params")
        self.settings.setValue("ip", self._widget.ip_box.text())
        self.settings.setValue("port", self._widget.port_box.text())
        self.settings.endGroup()
        try:
            response = requests.post(address,json=self.json_dict)
            self.instance_id = response.json()[u'instance_id']
        except:
            self._widget.launch_button.setStyleSheet("background-color: #F5A9A9")
            return
        self.is_remote = False
        self._widget.launch_button.setStyleSheet("background-color: #8fb8e0")
        self._widget.terminate_button.setStyleSheet("background-color: #FFFFFF")

    def _handle_config_ref_button_clicked(self):
        rospack = rospkg.RosPack()
        fname = QFileDialog.getOpenFileName(self._widget, 'Open file', rospack.get_path('lgsvl_simulator_bridge')+'/config')
        self._widget.configpath_box.setText(fname[0])

    def _handle_config_button_clicked(self):
        path = self._widget.configpath_box.text()
        try:
            f = open(path, "r+")
            self.json_dict = json.load(f)
            self.settings.beginGroup("simulator_params")
            self.settings.setValue("config_path", path)
            self.settings.endGroup()
        except:
            self._widget.button_config.setStyleSheet("background-color: #F5A9A9")
            return
        self._widget.button_config.setStyleSheet("background-color: #8fb8e0")

    def _handle_terminate_button_clicked(self):
        address = "http://" + self._widget.ip_box.text() + ":" + self._widget.port_box.text()+"/simulator/terminate"
        self.settings.beginGroup("simulator_params")
        self.settings.setValue("ip", self._widget.ip_box.text())
        self.settings.setValue("port", self._widget.port_box.text())
        self.settings.endGroup()
        if self.instance_id == "" and self.is_remote == True:
            return
        if self.is_remote == True:
            try:
                response = requests.post(address,json={u'instance_id':self.instance_id})    
            except:
                self._widget.terminate_button.setStyleSheet("background-color: #F5A9A9")
                return
        try:
            self.proc.terminate()
        except:
            self._widget.terminate_button.setStyleSheet("background-color: #F5A9A9")
            return            
        self._widget.terminate_button.setStyleSheet("background-color: #8fb8e0")
        self._widget.launch_button.setStyleSheet("background-color: #FFFFFF")