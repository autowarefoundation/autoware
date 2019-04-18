from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from ..core import myutils



class AwProcessPanel(QtWidgets.QStackedWidget):

    def __init__(self, client):
        super(AwProcessPanel, self).__init__()
        self.__client = client
        self.__items  = {}
        self.__server = None

        self.__dummy = QtWidgets.QLabel("This is node")
        self.addWidget(self.__dummy)

        self.__tmpdir = "/tmp/autoware_launcher"
        myutils.makedirs(self.__tmpdir, mode=0o700, exist_ok=True)

    def profile_ui_cleared(self):
        for key in self.__items.keys():
            self.__items.pop(key).deleteLater()

    def node_ui_updated(self, lnode):
        pass

    def node_ui_created(self, lnode):
        lpath = lnode.path()
        if lnode.plugin().isleaf():
            item = AwProcessItem(lpath)
            self.__items[lpath] = item
            self.addWidget(item)

    #def config_removed(self, lpath):

    def config_selected(self, lpath):
        item = self.__items.get(lpath, self.__dummy)
        self.setCurrentWidget(item)

    def roslaunch(self, lpath, xtext):
        xpath = self.__tmpdir + "/" + lpath.replace("/", "-") + ".xml"
        with open(xpath, mode="w") as fp:
            fp.write(xtext)
        self.__items[lpath].proc.start("roslaunch {}".format(xpath))

    def terminate(self, lpath):
        self.__items[lpath].proc.terminate()

    def register_server(self, server):
        self.__server = server

    def runner_finished(self, lpath):
        self.__server.runner_finished(lpath)



class AwProcessItem(QtWidgets.QPlainTextEdit):

    def __init__(self, lpath):
        super(AwProcessItem, self).__init__()
        self.lpath = lpath
        
        #ToDo: setMaximumBlockCount
        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)

        self.proc = QtCore.QProcess(self)
        self.proc.finished.connect(self.proc_finished)
        self.proc.readyReadStandardOutput.connect(self.proc_stdouted)
        self.proc.readyReadStandardError.connect(self.proc_stderred)

        import re
        self.bash_regex = re.compile("\033(\[.*?m|\].*?;)")

    def proc_finished(self):
        self.parent()._AwProcessPanel__server.runner_finished(self.lpath)

    def proc_stdouted(self):
        byte = self.proc.readAllStandardOutput()
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    def proc_stderred(self):
        byte = self.proc.readAllStandardError()
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)
