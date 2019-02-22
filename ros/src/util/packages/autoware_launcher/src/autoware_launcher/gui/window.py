from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core import myutils



class AwAbstructWindow(QtWidgets.QMainWindow):

    def __init__(self, parent):
        super(AwAbstructWindow, self).__init__(parent)

    def load_geomerty(self):
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

    def save_geometry(self):
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())



class AwMainWindow(AwAbstructWindow):

    def __init__(self, client):

        super(AwMainWindow, self).__init__(None)
        self.client = client

        self.load_geomerty()
        self.setWindowTitle("Autoware Launcher")

        self.__init_menu()

    def closeEvent(self, event):

        self.save_geometry()
        super(AwMainWindow, self).closeEvent(event)

    def __init_menu(self):

        self.filemenu = self.menuBar().addMenu("File")
        self.viewmenu = self.menuBar().addMenu("View")

        load_action = QtWidgets.QAction("Load Profile", self)
        load_action.setShortcut("Ctrl+L")
        load_action.triggered.connect(self.load_profile)

        save_action = QtWidgets.QAction("Save Profile", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_profile)

        save_as_action = QtWidgets.QAction("Save Profile As", self)
        save_as_action.setShortcut("Ctrl+A")
        save_as_action.triggered.connect(self.save_profile_as)

        export_action = QtWidgets.QAction("Export Profile", self)
        export_action.setShortcut("Ctrl+E")
        export_action.triggered.connect(self.export_profile)

        self.filemenu.addAction(load_action)
        self.filemenu.addAction(save_action)
        self.filemenu.addAction(save_as_action)
        self.filemenu.addAction(export_action)

    def addViewMenu(self, text, func):
        action = QtWidgets.QAction(text, self)
        action.setCheckable(True)
        action.toggled.connect(func)
        self.viewmenu.addAction(action)

    def load_profile(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Load Profile", myutils.profile(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            self.client.load_profile(filename)

    def save_profile(self):
        pass

    def save_profile_as(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getSaveFileName(self, "Save Profile As", myutils.profile(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            if filetype != ".launch":
                filename = filename + filetype
            self.client.save_profile(filename)
    
    def export_profile(self):
        import os
        dirname = QtWidgets.QFileDialog.getExistingDirectory(self, "Export Profile", myutils.package("launch"))
        if dirname:
            self.client.export_profile(dirname)