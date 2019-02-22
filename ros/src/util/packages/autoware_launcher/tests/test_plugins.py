from nose.tools import ok_
from autoware_launcher.core.plugin import AwPluginTree
from autoware_launcher.gui.guimgr  import AwQtGuiManager

def test_plugins():
    guimgr = AwQtGuiManager(None)
    plugins = AwPluginTree()
    for plugin_path in plugins.scan(""):
        plugin = plugins.find(plugin_path)
        fields = plugin.fields()
        for frame in plugin.panel().frames:
            print plugin.path() + " " + str(frame.target)
            yield ok_, guimgr.widget(frame).validate_argtypes(fields, frame)
            print
