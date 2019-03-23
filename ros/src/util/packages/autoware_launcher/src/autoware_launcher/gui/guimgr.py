from logging import getLogger
logger = getLogger(__name__)

import importlib
import os

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core import myutils

# ToDo: move package
#     core : client, mirror
#     gui  : guimgr

class AwQtGuiManager(object):

    def __init__(self, client):
        self.__client  = client
        self.__widgets = {}

        import autoware_launcher.gui.plugins as plugins
        for modkey, modcls in plugins.modules.items():
            for guikey, guicls in modcls.plugin_widgets().items():
                logger.debug("load plugin module: " + modkey + "." + guikey)
                self.__widgets[modkey + "." + guikey] = guicls

    def client(self):
        return self.__client

    def widget(self, view):
        return self.__widgets[view.widget]

    def create_widget(self, node, view, parent = None, widget = None):
        widget = widget or self.widget(view)
        return widget(self, node, view)

    def create_frame(self, mirror, guikey = None, guicls = None):
        #logger.debug("Create Frame: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls))
        if not guicls:
            guikey = guikey or mirror.plugin().frame()
            guicls = self.__widgets[guikey + "_frame"]
        return guicls(self, mirror)

    def create_panel(self, mirror, guikey = None, guicls = None):
        #logger.debug(reate Panel: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls))
        if not guicls:
            guikey = guikey or mirror.plugin().panel()
            guicls = self.__widgets[guikey + "_panel"]
        return guicls(self, mirror)

    def create_arg_frame(self, parent, opts):
        guicls = self.__widgets["args." + opts["type"]]
        return guicls(self, parent, opts)

    def create_info_frame(self, parent, opts):
        guicls = self.__widgets["info." + opts["type"]]
        return guicls(self, parent, opts)

    def create_frame_entire_vlayout(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        return layout

    def create_frame_header_hlayout(self):
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(5, 2, 2, 2)
        return layout

    def panel_add_widget(self, panel, widget):

        index = panel.layout().count() - 2
        panel.layout().insertWidget(index, widget)

    def panel_setup(self, widget, spacer = None):

        if widget.layout() is None:
            self.__panel_setup(widget, spacer)
        else:
            self.__panel_reset(widget)

    def __panel_setup(self, widget, spacer):

        footer_layout = QtWidgets.QHBoxLayout()
        footer_layout.setContentsMargins(2, 2, 2, 2)
        footer_layout.setSpacing(2)
        footer_layout.addStretch()
        widget.footer = QtWidgets.QWidget()
        widget.footer.setLayout(footer_layout)

        widget_layout = QtWidgets.QVBoxLayout()
        widget_layout.setContentsMargins(16, 16, 16, 16)
        widget_layout.setSpacing(16)
        if not spacer:
            widget_layout.addStretch()
        else:
            widget_layout.addWidget(spacer)
        widget_layout.addWidget(widget.footer)
        widget.setLayout(widget_layout)

    def __panel_reset(self, widget):

        footer_layout = widget.footer.layout()
        while 1 < footer_layout.count():
            footer_layout.takeAt(footer_layout.count() - 1).widget().deleteLater()

        widget_layout = widget.layout()
        while 2 < widget_layout.count():
            widget_layout.takeAt(0).widget().deleteLater()

    def frame_add_widget(self, frame, widget):
        widget.setObjectName("FrameWidget")
        frame.layout().addWidget(widget)

    def frame_add_button(self, frame, button):
        frame.header.layout().addWidget(button)

    def frame_add_text_widget(self, frame, text):
        layout = self.create_frame_header_hlayout()
        layout.addWidget(QtWidgets.QLabel(text))
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.frame_add_widget(frame, widget)

    def frame_setup(self, widget):

        if widget.layout() is None:
            self.__frame_setup(widget)
        else:
            self.__frame_reset(widget)

    def __frame_setup(self, widget):

        widget.title = QtWidgets.QLabel("No Title")
        widget.title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        layout = self.create_frame_header_hlayout()
        layout.addWidget(widget.title)
        widget.header = QtWidgets.QWidget()
        widget.header.setObjectName("FrameHeader")
        widget.header.setLayout(layout)

        layout = self.create_frame_entire_vlayout()
        layout.addWidget(widget.header)
        widget.setLayout(layout)

    def __frame_reset(self, widget):

        layout = widget.header.layout()
        while 1 < layout.count():
            layout.takeAt(layout.count() - 1).widget().deleteLater()

        layout = widget.layout()
        while 1 < layout.count():
            layout.takeAt(layout.count() - 1).widget().deleteLater()
