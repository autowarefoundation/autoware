from autoware_launcher.core import plugin
from autoware_launcher.core import myutils
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
import argparse
import collections
import re
import rospkg
import yaml
import xml.etree.ElementTree as xtree


def represent_ordered_dict(dumper, instance):
    return dumper.represent_mapping('tag:yaml.org,2002:map', instance.items())
yaml.add_representer(collections.OrderedDict, represent_ordered_dict)


def main(sys_argv):

    if len(sys_argv) < 2:
        print("missing plugin file path")
        return 2

    application = QtWidgets.QApplication(sys_argv)
    widget = PluginEditWidget()

    ynode = plugin.AwPluginNode(None, sys_argv[1])
    ynode.load(myutils.package("plugins"))
    xnode = load_launch_xml(ynode.rosxml())

    fields = [PluginEditField(child.attrib) for child in xnode if child.tag == "arg"]
    for field in fields: widget.add_field(field)
    widget.add_button()
    for field in ynode.args(): widget.set_yaml(field)

    widget.show()
    return application.exec_()


def load_launch_xml(rosxml):

    rospack = rospkg.RosPack()
    regex = re.compile("\$\(find (.*?)\)")
    match = regex.match(rosxml)
    xpath = regex.sub(rospack.get_path(match.group(1)), rosxml)
    return xtree.parse(xpath).getroot()


class PluginEditWidget(QtWidgets.QWidget):

    header = ["Field Name", "Field Type", "Array Type", "Default Value"]

    def __init__(self):

        super(PluginEditWidget, self).__init__()
        self.fields = collections.OrderedDict()
        self.setLayout(QtWidgets.QGridLayout())
        for col,text in enumerate(PluginEditWidget.header):
            self.layout().addWidget(QtWidgets.QLabel(text), 0, col)
        self.export = QtWidgets.QPushButton("Export")
        self.export.clicked.connect(self.export_yaml)

    def add_field(self, field):

        row = self.layout().rowCount()
        self.layout().addWidget(field.name,    row, 0)
        self.layout().addWidget(field.type,    row, 1)
        self.layout().addWidget(field.list,    row, 2)
        self.layout().addWidget(field.default, row, 3)
        self.fields[field.name.text()] = field

    def add_button(self):

        row = self.layout().rowCount()
        self.layout().addWidget(self.export, row, 0, 1, 4)

    def set_yaml(self, data):

        self.fields[data.name].set_yaml(data)

    def export_yaml(self):

        views  = [field.export_view() for field in self.fields.values()]
        fields = [field.export_dict() for field in self.fields.values()]
        data = collections.OrderedDict()
        data["args"] = fields
        data["panel"] = collections.OrderedDict()
        data["panel"]["widget"] = "node.panel"
        data["panel"]["frames"] = views
        text = yaml.dump(data, default_flow_style=None)
        print(text)
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(text)


class PluginEditField(object):

    viewtypes = {"str":"text", "int":"int", "real":"real", "bool":"bool"}

    def __init__(self, attr):

        self.name = QtWidgets.QLabel()
        self.type = QtWidgets.QComboBox()
        self.list = QtWidgets.QComboBox()
        self.default = QtWidgets.QLineEdit()

        self.name.setText(attr["name"])
        self.type.addItems(["str", "int", "real", "bool"])
        self.list.addItems(["none", "space", "yaml"])
        self.default.setText(attr.get("default"))

        self.type.setCurrentIndex(-1)
        if attr.get("default"):
            itype = self.type_inference(attr["default"])
            self.type.setCurrentIndex(self.type.findText(itype))

    def type_inference(self, value):

        if value.lower() in ["true", "false"]: return "bool"
        if value.isdigit(): return "int"
        if value.replace('.','',1).isdigit(): return "real"
        return "str"

    def set_yaml(self, data):

        self.type.setCurrentIndex(self.type.findText(data.type))
        if data.list:
            self.list.setCurrentIndex(self.list.findText(data.list))
        if data.rest.get("default"):
            self.default.setText(str(data.rest["default"]))

    def export_dict(self):

        data = collections.OrderedDict()
        data["name"] = str(self.name.text())
        data["type"] = str(self.type.currentText())
        if self.list.currentText() != "none":
            data["list"] = str(self.list.currentText())
        if self.default.text():
            data["default"] = self.export_default(data, self.default.text())
        return data

    def export_default(self, data, value):

        if data.get("list") is None:
            if data["type"] == "str" : return str(value)
            if data["type"] == "int" : return int(value)
            if data["type"] == "real": return float(value)
            if data["type"] == "bool": return (value.lower() == "true")
        raise NotImplementedError("Unkown Type: " + str(data))

    def export_view(self):

        data = collections.OrderedDict()
        data["target"] = "args." + str(self.name.text())
        data["widget"] = "basic." + PluginEditField.viewtypes[self.type.currentText()]
        return data