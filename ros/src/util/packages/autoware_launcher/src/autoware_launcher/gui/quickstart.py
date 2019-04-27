from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from ..core import myutils
from ..core import AwLaunchClientIF



class AwQuickStartPanel(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwQuickStartPanel, self).__init__()
        self.guimgr = guimgr
        self.frames = {"root/" + name: None for name in ["map", "vehicle", "sensing", "visualization"]}
        self.awlogo = QtWidgets.QLabel()

        pixmap = QtGui.QPixmap(myutils.package("resources/autoware_logo.png"))
        self.awlogo.setPixmap(pixmap)
        self.awlogo.setAlignment(QtCore.Qt.AlignCenter)
        self.awlogo.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)

    def profile_ui_cleared(self):
        self.guimgr.panel_setup(self, self.awlogo)

    def node_ui_created(self, lnode):
        if lnode.path() == "root": self.setup_widget(lnode)

    def node_ui_updated(self, lnode):
        if lnode.path() == "root": self.setup_widget(lnode)

    def status_ui_updated(self, lpath, state):
        frame = self.frames.get(lpath)
        if frame:
            if state == 0x00: frame.term_completed()
            if state == 0x01: frame.exec_requested()
            if state == 0x02: frame.term_requested()

    def setup_widget(self, node):
        self.guimgr.panel_setup(self, self.awlogo)
        for child in node.childnodes():
            if child.path() in self.frames:
                frame = self.guimgr.create_widget(child, child.plugin().frame(), widget = AwLaunchFrame)
                self.frames[child.path()] = frame
                self.guimgr.panel_add_widget(self, frame)



class AwLaunchFrame(QtWidgets.QWidget):

    def __init__(self, guimgr, node, opts):
        super(AwLaunchFrame, self).__init__()
        self.node = node
        self.states = ["Launch", "Terminate"]

        guimgr.frame_setup(self)

        self.title.setText(node.name().capitalize())
        guimgr.frame_add_text_widget(self, node.get_config("exts.description", "No Description"))

        self.button = QtWidgets.QPushButton(self.states[0])
        self.button.clicked.connect(self.onclicked)
        guimgr.frame_add_button(self, self.button)

    def onclicked(self):
        state_text = self.button.text()
        if state_text == self.states[0]:
            self.node.launch(True)
            self.exec_requested()
        if state_text == self.states[1]:
            self.node.launch(False)
            self.term_requested()

    def exec_requested(self):
        self.button.setText(self.states[1])

    def term_requested(self):
        self.button.setEnabled(False)

    def term_completed(self):
        self.button.setText(self.states[0])
        self.button.setEnabled(True)

