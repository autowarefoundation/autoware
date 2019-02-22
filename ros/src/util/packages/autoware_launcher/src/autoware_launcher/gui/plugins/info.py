from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.gui  import widgets



def plugin_widgets():
    return \
    {
        "node" : AwNodeInfoEdit,
    }



class AwNodeInfoEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwNodeInfoEdit, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwNodeInfoEdit, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.node.get_config("info.title", ""))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title("Node Title")

    def edited(self):
        self.node.update({"config": {"info.title": self.edit.text()}})

    @staticmethod
    def tostring(node, opts):
        return "{}: {}".format("Node Title", node.get_config("info.title"))