from rqt_gui_py.plugin import Plugin
import python_qt_binding.QtGui as QtGui
from python_qt_binding.QtGui import (QAction, QIcon, QMenu, QWidget,
                                     QPainter, QColor, QFont, QBrush, 
                                     QPen, QMessageBox, QSizePolicy,
                                     QDialog, QComboBox, QLineEdit, QCompleter, QPushButton)
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot, QEvent, QSize
import rospy
import python_qt_binding.QtCore as QtCore
from std_msgs.msg import Bool, Time
import math
from resource_retriever import get_filename
import yaml
import os, sys
from std_srvs.srv import Empty

class LineEditDialog(QDialog):
    def __init__(self, parent=None):
        super(LineEditDialog, self).__init__()
        self.value = None
        vbox = QtGui.QVBoxLayout(self)
        # combo box
        model = QtGui.QStandardItemModel(self)
        for elm in rospy.get_param_names():
            model.setItem(model.rowCount(), 0, QtGui.QStandardItem(elm))
        self.combo_box = QtGui.QComboBox(self)
        self.line_edit = QtGui.QLineEdit()
        self.combo_box.setLineEdit(self.line_edit)
        self.combo_box.setCompleter(QtGui.QCompleter())
        self.combo_box.setModel(model)
        self.combo_box.completer().setModel(model)
        self.combo_box.lineEdit().setText('')
        vbox.addWidget(self.combo_box)
        # button
        button = QPushButton()
        button.setText("Done")
        button.clicked.connect(self.buttonCallback)
        vbox.addWidget(button)
        self.setLayout(vbox)
    def buttonCallback(self, event):
        self.value = self.line_edit.text()
        self.close()

class ServiceButtonGeneralWidget(QWidget):
    """
    Qt widget to visualize multiple buttons
    """
    def __init__(self, button_type = "push"):
        super(ServiceButtonGeneralWidget, self).__init__()
        self.button_type = button_type
        self._layout_param = None
        self._dialog = LineEditDialog()
        self.show()
    def showError(self, message):
        QMessageBox.about(self, "ERROR", message)
    def loadLayoutYaml(self, layout_param):
        # Initialize layout of the buttons from yaml file
        # The yaml file can be specified by rosparam
        layout_yaml_file = rospy.get_param(
            layout_param,
            "package://jsk_rqt_plugins/resource/service_button_layout.yaml")
        resolved_layout_yaml_file = get_filename(layout_yaml_file)[len("file://"):]
        # check file exists
        if not os.path.exists(resolved_layout_yaml_file):
            self.showError("Cannot find %s (%s)" % (
                           layout_yaml_file, resolved_layout_yaml_file))
            sys.exit(1)
        self.setupButtons(resolved_layout_yaml_file)
        self.show()
    def setupButtons(self, yaml_file):
        """
        Parse yaml file and setup Buttons. Format of the yaml file should be:
        - name: 'button name' (required)
          image: 'path to image for icon' (optional)
          image_size: 'width and height of icon' (optional)
          service: 'service' (required)
          column: 'column index' (optional, defaults to 0)
        """
        self.buttons = []
        with open(yaml_file) as f:
            yaml_data = yaml.load(f)
            # lookup colum direction
            direction = 'vertical'
            for d in yaml_data:
                if d.has_key('direction'):
                    if d['direction'] == 'horizontal':
                        direction = 'horizontal'
                    else: # d['direction'] == 'vertical':
                        direction = 'vertical'
                    yaml_data.remove(d)
                    break
            # lookup column num
            column_indices = [d['column'] for d in yaml_data]
            max_column_index = max(*column_indices)
            if direction == 'vertical':
                self.layout = QtGui.QHBoxLayout()
                self.layout_boxes = [QtGui.QVBoxLayout()
                                     for i in range(max_column_index + 1)]
            else: # direction == 'horizontal'
                self.layout = QtGui.QVBoxLayout()
                self.layout_boxes = [QtGui.QHBoxLayout()
                                     for i in range(max_column_index + 1)]
            self.button_groups = [QtGui.QGroupBox()
                                 for i in range(max_column_index + 1)]
            for button_data in yaml_data:
                # check if all the field is available
                if not button_data.has_key("name"):
                    self.showError("name field is missed in yaml")
                    raise Exception("name field is missed in yaml")
                if not button_data.has_key("service"):
                    self.showError("service field is missed in yaml")
                    raise Exception("service field is missed in yaml")
                if self.button_type == "push":
                    button = QtGui.QToolButton()
                else: # self.button_type == "Radio":
                    button = QtGui.QRadioButton()
                button.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
                if button_data.has_key("image"):
                    image_file = get_filename(button_data["image"])[len("file://"):]
                    if os.path.exists(image_file):
                        icon = QtGui.QIcon(image_file)
                        button.setIcon(icon)
                        if button_data.has_key("image_size"):
                            button.setIconSize(QSize(button_data["image_size"][0], button_data["image_size"][1]))
                        else:
                            button.setIconSize(QSize(100, 100))
                if button_data.has_key("name"):
                    name = button_data['name']
                    button.setText(name)
                button.clicked.connect(self.buttonCallback(button_data['service']))
                if self.button_type == "push":
                    button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
                else: # self.button_type == "Radio":
                    if button_data.has_key("default_value") and button_data['default_value']:
                        button.setChecked(True)
                self.layout_boxes[button_data['column']].addWidget(button)
                self.buttons.append(button)
            for i in range(len(self.button_groups)):
                self.button_groups[i].setLayout(self.layout_boxes[i])
            for group in self.button_groups:
                self.layout.addWidget(group)
            self.setLayout(self.layout)
    def buttonCallback(self, service_name):
        """
        return function as callback
        """
        return lambda x: self.buttonCallbackImpl(service_name)
    def buttonCallbackImpl(self, service_name):
        srv = rospy.ServiceProxy(service_name, Empty)
        try:
            srv()
        except rospy.ServiceException, e:
            self.showError("Failed to call %s" % service_name)
    def save_settings(self, plugin_settings, instance_settings):
        if self._layout_param:
            instance_settings.set_value("layout_param", self._layout_param)
            rospy.loginfo("save setting is called. %s" % self._layout_param)
    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.value("layout_param"):
            self._layout_param = instance_settings.value("layout_param")
            self.loadLayoutYaml(self._layout_param)
            rospy.loginfo("restore setting is called. %s" % self._layout_param)
    def trigger_configuration(self):
        self._dialog.exec_()
        self._layout_param = self._dialog.value
        self.loadLayoutYaml(self._layout_param)
        rospy.loginfo("trigger configuration is called. %s" % self._dialog.value)
