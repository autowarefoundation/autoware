from logging import getLogger
logger = getLogger(__name__)

import requests
import yaml
from python_qt_binding import QtCore
from python_qt_binding import QtNetwork
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from autoware_launcher.core import myutils



class AwLgsvlSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwLgsvlSimulatorWidget, self).__init__()
        self.process = QtCore.QProcess(self)
        self.console = AwProcessViewer(self.process)
        self.button = QtWidgets.QPushButton("Launch Simulator")
        self.button.setCheckable(True)
        self.button.toggled.connect(self.launch_lgsvm)

        self.server_addr = QtWidgets.QLineEdit()
        self.server_port = QtWidgets.QLineEdit()
        self.client_addr = QtWidgets.QLineEdit()
        self.client_port = QtWidgets.QLineEdit()

        self.server_addr.setText("10.100.2.1")
        self.server_port.setText("5000")
        for host in QtNetwork.QNetworkInterface.allAddresses():
            if not host.isLoopback():
                if host.protocol() == QtNetwork.QAbstractSocket.IPv4Protocol:
                    self.client_addr.setText(host.toString())
        self.client_port.setText("9090")

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel("Server Address"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("Server Port"),    1, 0)
        layout.addWidget(QtWidgets.QLabel("Client Address"), 2, 0)
        layout.addWidget(QtWidgets.QLabel("Client Port"),    3, 0)
        layout.addWidget(self.server_addr, 0, 1)
        layout.addWidget(self.server_port, 1, 1)
        layout.addWidget(self.client_addr, 2, 1)
        layout.addWidget(self.client_port, 3, 1)
        layout.addWidget(self.button,      4, 0, 1, 2)
        layout.addWidget(self.console,     5, 0, 1, 2)
        self.setLayout(layout)

    def launch_lgsvm(self, checked):
        server_address = "http://{}:{}/simulator/".format(self.server_addr.text(), self.server_port.text())
        if checked:
            self.process.start("roslaunch rosbridge_server rosbridge_websocket.launch")
            with open(myutils.package("resources/lgsvl.yaml")) as fp:
                param = yaml.safe_load(fp)
            param["bin_type"] = "tier4-develop"
            param["vehicles"][0]["address"] = self.client_addr.text()
            param["vehicles"][0]["port"]    = self.client_port.text()
            responce = requests.post(server_address + "launch", json=param)
            logger.debug(responce.status_code)
            logger.debug(responce.json())
            self.instance = responce.json()["instance_id"]
        else:
            self.process.terminate()
            responce = requests.post(server_address + "terminate", json={"instance_id": self.instance})
            logger.debug(responce.status_code)
            logger.debug(responce.json())



class AwProcessViewer(QtWidgets.QPlainTextEdit):

    def __init__(self, process):
        super(AwProcessViewer, self).__init__()
        
        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        #self.setMaximumBlockCount(100)
        self.moveCursor(QtGui.QTextCursor.End)

        self.process = process
        self.process.finished.connect(self.process_finished)
        self.process.readyReadStandardOutput.connect(self.process_stdouted)
        self.process.readyReadStandardError.connect(self.process_stderred)

        import re
        self.bash_regex = re.compile("(\x1b\[.*?m|\x1b\]2;|\x07)")

    def byte2text(self, byte):
        #text = self.bash_regex.sub("", byte.data().decode('utf-8'))
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        #text = str(text).encode("string-escape").replace("\\n", "\n")
        return text

    def process_finished(self):
        pass

    def process_stdouted(self):
        text = self.byte2text(self.process.readAllStandardOutput())
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    def process_stderred(self):
        text = self.byte2text(self.process.readAllStandardError())
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    # Debug
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_C:
            self.clear()
            event.accept()
        else:
            super(AwProcessViewer, self).keyPressEvent(event)