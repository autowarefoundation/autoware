from logging import getLogger
logger = getLogger(__name__)

import os
import yaml

from . import basetree
from . import myutils



class AwBaseNode(object):

    def __init__(self, name):
        self.__nodename = name
        self.__parent   = None
        self.__children = []
        self.__childmap = {}

    def dump(self, indent = 0):
        print((indent * " ") + str(self))
        for child in self.children(): child.dump(indent + 2)

    def tree(self):
        return self.__parent.tree()

    def nodename(self): # ToDo: remove
        return self.__nodename
    
    def name(self):
        return self.__nodename

    def nodepath(self): # ToDo: remove
        return os.path.join(self.__parent.nodepath(), self.nodename())

    def path(self):
        return os.path.join(self.__parent.path(), self.name())

    def fullpath(self):
        return os.path.join(self.__parent.fullpath(), self.nodename())

    def children(self): # ToDo: remove
        return self.__children
    
    def childnodes(self):
        return self.__children

    def childnames(self):
        return [node.nodename() for node in self.__children]

    def getchild(self, name):
        return self.__childmap.get(name)

    def haschild(self, name):
        return name in self.__childmap

    def addchild(self, node):
        self.__children.append(node)
        self.__childmap[node.nodename()] = node
        node.__parent = self

    def delchild(self, node):
        self.__children.remove(node)
        self.__childmap.pop(node.nodename())
        node.__parent = None

    def listnode(self, this = False):
        result = [self] if this else []
        for child in self.children(): result.extend(child.listnode(True))
        return result



class AwBaseTree(AwBaseNode):

    def __init__(self):
        super(AwBaseTree, self).__init__(None)
        self.treepath = ""

    def tree(self):
        return self

    def nodepath(self): # ToDo: remove
        return ""

    def path(self):
        return ""
    
    def fullpath(self):
        return self.treepath

    def find(self, path):
        node = self
        for name in path.split("/"):
            node = node.getchild(name)
        return node



class AwLaunchTree(AwBaseTree):

    def __init__(self, server, plugins):
        super(AwLaunchTree, self).__init__()
        self.server  = server
        self.plugins = plugins
        #self.treedir = None

    def __str__(self):
        childnames = map(lambda child: child.nodename(), self.children())
        return "Tree:{} Children:{}".format(self.nodename(), childnames)

    def save(self, treepath):
        self.treepath = treepath
        with open(treepath + ".launch", mode = "w") as fp:
            fp.write("dummy")
        for node in self.listnode():
            fullpath = node.fullpath() + ".yaml"
            myutils.makedirs(os.path.dirname(fullpath), exist_ok = True)
            with open(fullpath, mode = "w") as fp:
                fp.write(yaml.safe_dump(node.export_data(), default_flow_style = False))

    def load(self, treepath, plugins):
        def load_node(node):
            fullpath = node.fullpath()
            with open(fullpath + ".yaml") as fp:
                node.import_data(yaml.safe_load(fp), plugins)
            for child in node.children():
                load_node(child)
        root = AwLaunchNode("root")
        self.addchild(root)
        self.treepath = treepath
        load_node(root)

    def make(self, ppath, plugins):
        plugin = plugins.find(ppath)
        launch = AwLaunchNode("root")
        launch.plugin = plugin
        launch.config = plugin.default_config()
        self.addchild(launch)
    
    def export(self, rootpath):
        for node in self.listnode():
            xtext = node.generate_launch()
            xpath = node.nodepath().replace("/", "-") + ".xml"
            xpath = os.path.join(rootpath, xpath)
            with open(xpath, mode="w") as fp: fp.write(xtext)

    def create(self, lpath, ppath):
        logger.debug("Tree Create: " + lpath + ", " + ppath)
        parent = self.find(os.path.dirname(lpath))
        if not parent:
            return "parent is not found"
        if self.find(lpath):
            return "name exists"

        plugin = self.plugins.find(ppath)
        launch = AwLaunchNode(os.path.basename(lpath))
        launch.plugin = plugin
        launch.config = plugin.default_config()
        parent.addchild(launch)
        return None




class AwLaunchNode(AwBaseNode):

    STOP, EXEC, TERM = 0x00, 0x01, 0x02

    def __init__(self, name):
        super(AwLaunchNode, self).__init__(name)
        self.plugin = None
        self.config = None
        self.status = self.STOP

    def tostring(self):
        return yaml.safe_dump(self.todict())

    def todict(self):
        return \
        {
            "plugin"  : self.plugin.todict(),
            "config"  : self.config,
            "children": [child.nodename() for child in self.children()]
        }

    # experimental
    def remove_child(self, name):
        if not name:
            return "name is empty"
        if not self.haschild(name):
            return "name does not exist"
        self.delchild(name)
        self.send_config_removed(name)
        return None

    def update(self, ldata):
        self.config.update(ldata["config"])
        return None

    def launch(self, xmode):
        if xmode:
            return self.__exec()
        else:
            return self.__term()

    def __exec(self):
        if self.plugin.isleaf():
            if self.status == self.STOP:
                self.status = self.EXEC
                return (True, True)
        else:
            status = self.STOP
            for child in self.children():
                status |= child.status
            if self.status != status:
                self.status = status
                return (True, False)
        return (False, False)

    def __term(self):
        if self.plugin.isleaf():
            if self.status == self.EXEC:
                self.status = self.TERM
                return (True, True)
        else:
            status = self.STOP
            for child in self.children():
                status |= child.status
            if self.status != status:
                self.status = status
                return (True, False)
        return (False, False)

    def get_config(self, key, value):
        return self.config.get(key, value)

    def generate_launch(self):
        lines = []
        if self.plugin.isleaf():
            lines.append('<launch>')
            lines.append('  <include file="{}">'.format(self.plugin.rosxml()))
            for data in self.plugin.args():
                argvalue = self.config.get("args." + data.name)
                if argvalue is not None:
                    lines.append('    <arg name="{}" value="{}"/>'.format(data.name, data.xmlstr(argvalue)))
            lines.append('  </include>')
            lines.append('</launch>')
        else:
            lines.append('<launch>')
            for childname in self.childnames():
                childpath = os.path.join(self.path(), childname)
                childpath = childpath.replace("/", "-") + ".xml"
                lines.append('  <include file="{}"/>'.format(childpath))
            lines.append('</launch>')
        return "\n".join(lines)



    def import_data(self, data, plugins):
        self.plugin = plugins.find(data["plugin"])
        self.config = data["config"]
        if data["children"] is None:
            self.setleaf()
        else:
            for childname in data["children"]:
                self.addchild(AwLaunchNode(childname))

    def export_data(self):
        children = map(lambda node: node.nodename(), self.children())
        plugin = self.plugin.path()
        config = self.config
        return { "children": children, "plugin": plugin, "config": config }


    # ToDo: remove function
    def bind_listener(self, listener):
        logger.warning("bind_listener: " + listener.__class__.__name__)

    # ToDo: remove function
    def unbind_listener(self, listener):
        logger.warning("unbind_listener: " + listener.__class__.__name__)



if __name__ == "__main__":
    from .plugin import AwPluginTree
    plugin = AwPluginTree()
    launch = AwLaunchTree(None)
    launch.load(myutils.profile("default"), plugin)
    launch.dump()
    launch.save(myutils.profile("sample.bak"))