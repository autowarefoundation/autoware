from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core import myutils


class AwAbstructPanel(QtWidgets.QWidget):

    def __init__(self, guimgr, node, view):
        super(AwAbstructPanel, self).__init__()
        self.guimgr = guimgr
        self.node = node
        self.view = view

    def setup_widget(self):
        if self.layout() is None:
            self.__setup_widget()
        else:
            self.__clear_widget()

    def __setup_widget(self):

        self.window().setWindowTitle(self.__class__.__name__)

        # Panel Footer
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        layout.addStretch()
        self.footer = QtWidgets.QWidget()
        self.footer.setLayout(layout)

        # Panel Layout
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        layout.addStretch()
        layout.addWidget(self.footer)
        self.setLayout(layout)

    def __clear_widget(self):

        # Panel Footer
        layout = self.footer.layout()
        while 1 < layout.count():
            layout.takeAt(layout.count() - 1).widget().deleteLater()

        # Panel Layout
        layout = self.layout()
        while 2 < layout.count():
            layout.takeAt(0).widget().deleteLater()

    def add_frame(self, frame):
        index = self.layout().count() - 2
        self.layout().insertWidget(index, frame)

    def add_button(self, button):
        self.footer.layout().addWidget(button)



class AwAbstructFrame(QtWidgets.QWidget):

    def __init__(self, guimgr, node, view):
        super(AwAbstructFrame, self).__init__()
        self.guimgr = guimgr
        self.node = node
        self.view = view

    def setup_widget(self):
        if self.layout() is None:
            self.__setup_widget()
        else:
            self.__clear_widget()

    def __setup_widget(self):

        # Frame Header
        self.title = QtWidgets.QLabel("No Title")
        self.title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        layout = self.guimgr.create_frame_header_hlayout()
        layout.addWidget(self.title)
        self.header = QtWidgets.QWidget()
        self.header.setObjectName("FrameHeader")
        self.header.setLayout(layout)

        # Frame Layout
        layout = self.guimgr.create_frame_entire_vlayout()
        layout.addWidget(self.header)
        self.setLayout(layout)

    def __clear_widget(self):

        # Frame Header
        layout = self.header.layout()
        while 1 < layout.count():
            layout.takeAt(layout.count() - 1).widget().deleteLater()

        # Frame Layout
        layout = self.layout()
        while 1 < layout.count():
            layout.takeAt(layout.count() - 1).widget().deleteLater()
    
    def set_title(self, title):
        self.title.setText(title)

    def add_button(self, button):
        self.header.layout().addWidget(button)

    def add_widget(self, widget):
        widget.setObjectName("FrameWidget")
        self.layout().addWidget(widget)

    def add_text_widget(self, text):
        layout = self.guimgr.create_frame_header_hlayout()
        layout.addWidget(QtWidgets.QLabel(text))
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.add_widget(widget)



class AwFileSelect(QtCore.QObject):

    def __init__(self, parent):
        super(AwFileSelect, self).__init__(parent)
        self.path   = QtWidgets.QLineEdit()
        self.button = QtWidgets.QPushButton("Browse")
        self.button.clicked.connect(self.browse)

    def browse(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self.button, "Select File", myutils.userhome())
        if filepath:
            self.path.setText(filepath)