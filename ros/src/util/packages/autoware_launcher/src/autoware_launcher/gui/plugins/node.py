from logging import getLogger
logger = getLogger(__name__)

from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.gui import widgets



def plugin_widgets():
    return \
    {
        "panel" : AwDefaultNodePanel,
        "frame" : AwDefaultNodeFrame,
    }



class AwDefaultNodePanel(widgets.AwAbstructPanel):

    def __init__(self, guimgr, node, view):
        super(AwDefaultNodePanel, self).__init__(guimgr, node, view)
        self.config = self.node.config().copy()

    def setup_widget(self):
        super(AwDefaultNodePanel, self).setup_widget()

        # For Debug
        self.debug1 = QtWidgets.QLabel(self.node.tostring())
        self.debug1.setVisible(False)
        self.add_frame(self.debug1)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)

        # For Debug
        self.debug2 = QtWidgets.QLabel(self.node.generate_launch())
        self.debug2.setVisible(False)
        self.add_frame(self.debug2)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)


        # data view
        for view in self.node.plugin().panel().frames:
            self.add_frame(self.guimgr.create_widget(self.node, view))

        # node view
        for rule in self.node.plugin().rules():
            if rule.unique:
                if self.node.haschild(rule.name):
                    child_node = self.node.getchild(rule.name)
                    child_view = child_node.plugin().frame()
                    self.add_frame(self.guimgr.create_widget(child_node, child_view))
                else:
                    self.add_frame(AwNodeCreateButton(self.node, rule, "Create " + rule.name.capitalize()))
            else:
                rule_names = [name for name in self.node.childnames() if name.startswith(rule.name)]
                for rule_name in rule_names:
                    child_node = self.node.getchild(rule_name)
                    child_view = child_node.plugin().frame()
                    self.add_frame(self.guimgr.create_widget(child_node, child_view))
                self.add_frame(AwNodeCreateButton(self.node, rule, "Create " + rule.name.capitalize()))

    # Debug
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_D:
            self.debug1.setVisible(not self.debug1.isVisible())
            event.accept()
        elif event.key() == QtCore.Qt.Key_F:
            self.debug2.setVisible(not self.debug2.isVisible())
            event.accept()
        elif event.key() == QtCore.Qt.Key_I:
            self.node.update({"config": self.node.plugin().default_config()})
            event.accept()
        else:
            super(AwDefaultNodePanel, self).keyPressEvent(event)



class AwDefaultNodeFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwDefaultNodeFrame, self).__init__(guimgr, node, view)

        super(AwDefaultNodeFrame, self).setup_widget()
        self.set_title(self.node.name().capitalize())
        self.add_button(AwConfigButton(self.guimgr.client(), self.node.path()))

        description = self.node.get_config("exts.description")
        if description:
            description = description.capitalize()
        else:
            config = node.config()
            description = []
            #for data in self.node.plugin().info():
            #    description.append(self.guimgr.widget(data).tostring(self.node, data))
            #for data in self.node.plugin().args():
            #    description.append(self.guimgr.widget(data).tostring(self.node, data))
            description = "\n".join(description) if description else "No Description"

        self.add_text_widget(description)


class AwConfigButton(QtWidgets.QPushButton):

    def __init__(self, client, lpath):
        super(AwConfigButton, self).__init__("Config")
        self.clicked.connect(lambda: client.select_config(lpath))



class AwNodeCreateButton(QtWidgets.QPushButton):

    def __init__(self, node, rule, text):

        super(AwNodeCreateButton, self).__init__(text)
        self.node = node
        self.rule = rule
        self.clicked.connect(self.onclicked)

    def newname(self):

        if self.rule.unique:
            return self.rule.name
        else:
            index = 0
            while self.node.haschild(self.rule.name + str(index)):
                index = index + 1
            return self.rule.name + str(index)

    def onclicked(self):

        if len(self.rule.plugins) == 1:
            self.node.addchild(self.newname(), self.rule.plugins[0])
        else:
            self.show_select_window()

    def onselected(self):

        items = self.ui_pname.selectedItems()
        if not items:
            self.ui_error.setText("Node type is not selected")
            return

        error = self.node.addchild(self.newname(), items[0].text())
        if error:
            self.ui_error.setText(error)
            return

        self.ui_error.setText("")
        self.ui_window.close()

    def show_select_window(self):

        # window
        window = QtWidgets.QMainWindow(self)
        window.setCentralWidget(QtWidgets.QWidget())
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.setGeometry(self.window().geometry())

        # widget
        window.setWindowTitle("Create Node")
        widget = window.centralWidget()
        widget.setLayout(QtWidgets.QVBoxLayout())

        # plugin select
        pname_select = QtWidgets.QListWidget()
        for pname in self.rule.plugins:
            pname_select.addItem(pname)
        widget.layout().addWidget(QtWidgets.QLabel("Node Type"))
        widget.layout().addWidget(pname_select)

        # footer
        error_label = QtWidgets.QLabel()
        error_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        cancel_button = QtWidgets.QPushButton("Cancel")
        select_button = QtWidgets.QPushButton("Select")
        cancel_button.clicked.connect(window.close)
        select_button.clicked.connect(self.onselected)
        footer = QtWidgets.QHBoxLayout()
        footer.addWidget(error_label)
        footer.addWidget(cancel_button)
        footer.addWidget(select_button)
        widget.layout().addLayout(footer)

        self.ui_window = window
        self.ui_error  = error_label
        self.ui_pname  = pname_select
        window.show()



#experimental
class AwPluginRemoveWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, launch, parent):
        super(AwPluginRemoveWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.node = launch

        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

        # select
        self.nodelist = QtWidgets.QListWidget()
        for child in self.node.children():
            self.nodelist.addItem(child.name())

        # footer
        cancel = QtWidgets.QPushButton("Cancel")
        cancel.clicked.connect(self.close)
        remove = QtWidgets.QPushButton("Remove")
        remove.clicked.connect(self.remove_launch_node)
        footer = QtWidgets.QHBoxLayout()
        footer.addStretch()
        footer.addWidget(cancel)
        footer.addWidget(remove)

        # widget
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.nodelist)
        layout.addLayout(footer)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.setWindowTitle("Remove Launch Node")

    def remove_launch_node(self):
        items = self.nodelist.selectedItems()
        if len(items) != 1:
            logger.error("node is not selected")
        else:
            error = self.node.remove_child(items[0].text())
            if error:
                logger.error(error)
            else:
                self.close()