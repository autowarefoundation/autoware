import argparse
import rospkg
from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

def main(sys_argv):

    parser = argparse.ArgumentParser(description='Autoware Launcher Plugin Editor')
    parser.add_argument("plugin", help="plugin file path")
    args = parser.parse_args()

    application = QtWidgets.QApplication(sys_argv)
    wizard = QtWidgets.QWizard()
    wizard.setWindowTitle("Plugin Editor")
    wizard.addPage(LaunchFileSelect())
    wizard.show()
    return application.exec_()

class LaunchFileSelectUI(object):

    def __init__(self, widget):

        self.pkgbox = QtWidgets.QComboBox()
        self.pkgbox.setEditable(True)
        self.pkgbox.setInsertPolicy(QtWidgets.QComboBox.NoInsert)

        self.xmlbox = QtWidgets.QComboBox()
        self.xmlbox.setEditable(True)

        widget.setTitle("Launch File Select")
        widget.setLayout(QtWidgets.QFormLayout())
        widget.layout().addRow("Package", self.pkgbox)
        widget.layout().addRow("File",    self.xmlbox)

class LaunchFileSelect(QtWidgets.QWizardPage):

    def __init__(self):

        super(LaunchFileSelect, self).__init__()
        self.rospack = rospkg.RosPack()
        self.pkglist = self.rospack.list()
        self.pkgname = "autoware_launcher"
        
        self.ui = LaunchFileSelectUI(self)
        for package in sorted(self.pkglist):
            self.ui.pkgbox.addItem(package)
        self.ui.pkgbox.setCurrentIndex(self.ui.pkgbox.findText(self.pkgname))
        self.ui.pkgbox.editTextChanged.connect(self.package_updated)

    def update_xmlbox(self, xmlbox):
        for filepath in sorted(["aaa", "bbb", "ccc"]):
            xmlbox.addItem(filepath)

    def package_updated(self, pkgname):

        if pkgname in self.pkglist:
            if self.pkgname != pkgname:
                self.pkgname = pkgname
                pkgpath = self.rospack.get_path(pkgname)
                print("Update: " + pkgpath)