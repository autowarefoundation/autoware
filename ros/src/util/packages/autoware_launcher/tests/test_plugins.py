from nose.tools import ok_
from autoware_launcher.core.plugin import AwPluginTree
from autoware_launcher.gui.guimgr  import AwQtGuiManager
import rospkg
import re
import xml.etree.ElementTree as xtree

def test_plugin_type():
    guimgr = AwQtGuiManager(None)
    plugins = AwPluginTree()
    for plugin_path in plugins.scan(""):
        plugin = plugins.find(plugin_path)
        fields = plugin.fields()
        for frame in plugin.panel().frames:
            print(plugin.path() + " " + str(frame.target))
            yield ok_, guimgr.widget(frame).validate_argtypes(fields, frame)
            print("")

def test_plugin_field():
    rospack = rospkg.RosPack()
    regexp = re.compile("\$\(find (.*?)\)(.*)")
    plugins = AwPluginTree()
    for plugin_path in plugins.scan(""):
        plugin = plugins.find(plugin_path)
        if plugin.isleaf():
            result = regexp.match(plugin.rosxml())
            try:
                path = rospack.get_path(result.group(1)) + result.group(2)
                root = xtree.parse(path).getroot()
                xmlargs = {child.attrib["name"] for child in root if child.tag == "arg"}
                ymlargs = {arg.name for arg in plugin.args()}
                print(plugin.path())
                yield ok_, (xmlargs == ymlargs)
                print("")
                if xmlargs != ymlargs:
                    print(ymlargs)
                    print(xmlargs)
            except:
                print(result.group(1) + " is not built")