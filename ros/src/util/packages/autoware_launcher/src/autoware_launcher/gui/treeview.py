import os

from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwTreeViewPanel(QtWidgets.QTreeWidget):

    def __init__(self, client):
        super(AwTreeViewPanel, self).__init__()
        self.__client = client
        self.__items  = {}
        self.__panels = []

        self.setColumnCount(2)
        self.setHeaderLabels(["Node", "Run Status"])
        self.header().setStretchLastSection(False)
        self.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.header().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)

        #self.addTopLevelItem(self.construct(launch))
        #self.expandToDepth(0)
        #self.itemChanged.connect(self.on_item_changed)
        self.currentItemChanged.connect(self.item_selectd)

    def keyPressEvent(self, event):
        if event.key() in [QtCore.Qt.Key_Up, QtCore.Qt.Key_Down, QtCore.Qt.Key_Left, QtCore.Qt.Key_Right]:
            super(AwTreeViewPanel, self).keyPressEvent(event)
        elif event.key() == QtCore.Qt.Key_R:
            lpath = self.currentItem().lpath
            self.__client.launch_node(lpath, True)
            event.accept()
        elif event.key() == QtCore.Qt.Key_T:
            lpath = self.currentItem().lpath
            self.__client.launch_node(lpath, False)
            event.accept()
        else:
            event.ignore()

    def select_config(self, lpath):
        self.setCurrentItem(self.__items[lpath])

    def register_select_listener(self, panel):
        self.__panels.append(panel)

    def status_ui_updated(self, lpath, state):
        self.__items[lpath].status_updated(state)

    def profile_ui_cleared(self):
        if self.__items:
            self.config_removed("root")

    def node_ui_updated(self, lnode):
        pass

    def node_ui_created(self, lnode):
        lpath = lnode.path()
        ppath, lname = os.path.split(lpath)
        if ppath:
            item = AwTreeViewItem(lpath)
            self.__items[lpath] = item
            self.__items[ppath].addChild(item)
        else:
            item = AwTreeViewItem(lpath)
            self.__items[lpath] = item
            self.addTopLevelItem(item)

    def config_removed(self, lpath):
        ppath, lname = os.path.split(lpath)
        item = self.__items[lpath]
        if ppath:
            parent = self.__items[ppath]
            parent.takeChild(parent.indexOfChild(item))
        else:
            self.takeTopLevelItem(self.indexOfTopLevelItem(item))

        for key in self.__items.keys():
            if key.startswith(lpath):
                self.__items.pop(key)

    def item_selectd(self, item, prev):
        if item:
            for panel in self.__panels:
                panel.config_selected(item.lpath)



class AwTreeViewItem(QtWidgets.QTreeWidgetItem):

    def __init__(self, lpath):
        super(AwTreeViewItem, self).__init__()
        self.lpath = lpath

        self.setText(0, os.path.basename(lpath))
        self.setText(1, "stop")

    def status_updated(self, state):
        self.setText(1, "unknown")
        if state == 0: self.setText(1, "stop")
        if state == 1: self.setText(1, "running")
        if state == 2: self.setText(1, "terminating")


class AwControlPanel(QtWidgets.QWidget):

    def __init__(self, client):
        super(AwControlPanel, self).__init__()
        self.__client = client

        self.__exec_button = QtWidgets.QPushButton("Exec")
        self.__term_button = QtWidgets.QPushButton("Term")
        self.__exec_button.clicked.connect(self.exec_config)
        self.__term_button.clicked.connect(self.term_config)

        self.__lpath = QtWidgets.QLabel()
        self.setLayout(QtWidgets.QVBoxLayout())

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(QtWidgets.QLabel("Path: "))
        layout.addWidget(self.__lpath)
        self.__lpath.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.layout().addLayout(layout)

        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.__exec_button)
        layout.addWidget(self.__term_button)
        self.layout().addLayout(layout)

    def exec_config(self):
        self.__client.launch_node(self.__lpath.text(), True)

    def term_config(self):
        self.__client.launch_node(self.__lpath.text(), False)

    def config_selected(self, lpath):
        self.__lpath.setText(lpath)