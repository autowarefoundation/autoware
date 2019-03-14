import os

from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwSummaryPanel(QtWidgets.QStackedWidget):

    def __init__(self, client):
        super(AwSummaryPanel, self).__init__()
        self.__client = client
        self.__panels = {}

    def profile_ui_cleared(self):
        for key in self.__panels.keys():
            self.__panels.pop(key).deleteLater()

    def node_ui_created(self, lnode):
        panel = self.__client.guimgr().create_widget(lnode, lnode.plugin().panel())
        panel.setup_widget()
        #self.__panels[lnode.path()] = panel
        #self.addWidget(panel)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(panel)
        scroll.setWidgetResizable(True)
        self.__panels[lnode.path()] = scroll
        self.addWidget(scroll)

    def node_ui_updated(self, lnode):
        self.__panels[lnode.path()].widget().setup_widget()

    #def config_removed(self, lpath):

    def config_selected(self, lpath):
        self.setCurrentWidget(self.__panels[lpath])


