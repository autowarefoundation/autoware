from logging import getLogger
logger = getLogger(__name__)

import os
import yaml

from . import basetree
from . import myutils



class AwPluginTree(basetree.AwBaseTree):

    def __init__(self):
        super(AwPluginTree, self).__init__()

        filelist = myutils.listfiles(myutils.plugins(), relative=True)
        nodelist = list()
        for filepath in filelist:
            fkey, fext = os.path.splitext(filepath)
            if fext in [".yaml", ".xml"]:
                if fkey not in self.nodes:
                    self.nodes[fkey] = AwPluginNode(self, fkey)
            else:
                logger.warning("plugin ignored: unknown extension ({})".format(filepath))

        for plugin in self.nodes.values():
            plugin.load(myutils.plugins())



class AwPluginNode(basetree.AwBaseNode):

    def __init__(self, tree, path):
        super(AwPluginNode, self).__init__(tree, path)
        self.__isleaf = None
        self.__rosxml = None
        self.__fields = None
        self.__args   = None
        self.__exts   = None
        self.__rules  = None
        self.__panel  = None
        self.__frame  = None

    def dump(self):
        print(yaml.safe_dump(self.todict()))

    def todict(self):
        return \
        {
            "1.name" : self.path(),
            "2.type" : "Node" if self.isnode() else "Leaf",
            "3.file" : self.__rosxml,
            "4.exts" : [data.todict() for data in self.__exts],
            "5.args" : [data.todict() for data in self.__args],
            "6.rules": [data.todict() for data in self.__rules],
            "7.panel": self.__panel.todict(),
            "8.frame": self.__frame.todict()
        }

    def isnode(self):
        return not self.__isleaf

    def isleaf(self):
        return self.__isleaf

    def rosxml(self):
        return self.__rosxml

    def fields(self):
        return self.__fields

    def exts(self):
        return self.__exts

    def args(self):
        return self.__args

    def rules(self):
        return self.__rules

    def panel(self):
        return self.__panel

    def frame(self):
        return self.__frame

    def default_config(self):
        values = {"str":"", "int":0, "real":0.0, "bool":False}
        def default_value(data):
            value = data.rest.get("default")
            if value is None:
                return values[data.type]
            else:
                return value
        config = {}
        config.update({"args." + data.name: default_value(data) for data in self.__args})
        config.update({"exts." + data.name: default_value(data) for data in self.__exts})
        return config

    def argstr(self, config):
        lines = []
        for argkey, argdef in self.__args.items():
            cfgkey = "args." + argkey
            lines.append(argkey + ": " + config[cfgkey])
        return "\n".join(lines)

    def load(self, rootpath):
        filepath = os.path.join(rootpath, self.path())

        with open(filepath+ ".yaml") as fp:
            ydata = yaml.safe_load(fp)

        if ydata.get("format") != "Autoware Launcher Plugin Version 0.1":
            raise Exception("unknown plugin format: " + filepath)

        self.__isleaf = ydata.get("rules") is None
        self.__rosxml = ydata.get("rosxml", "$(find autoware_launcher)/plugins/{}.xml".format(self.path()))
        self.__exts   = [AwPluginDataElement(data) for data in ydata.get("exts", [])]
        self.__args   = [AwPluginDataElement(data) for data in ydata.get("args", [])]
        self.__rules  = [AwPluginRuleElement(data, self) for data in ydata.get("rules", [])]
        self.__panel  = AwPluginPanelElement(ydata.get("panel", {}))
        self.__frame  = AwPluginFrameElement(ydata.get("frame", {}))
        self.__fields = {}
        self.__fields.update({"exts."+data.name: data for data in self.__exts})
        self.__fields.update({"args."+data.name: data for data in self.__args})



# AwPluginArgumentDataElement
# AwPluginExtendedDataElement
class AwPluginDataElement(object):

    def __init__(self, data):
        self.name = data.pop("name")
        self.type = data.pop("type")
        self.list = data.pop("list", None)
        self.rest = data # temporary, add default field

    def todict(self):
        return vars(self)

    def xmlstr(self, value):
        if self.list is None:
            return value
        if self.list == "space":
            return " ".join(value)
        if self.list == "yaml":
            return "[{}]".format(",".join(value))
        raise Error(__class__.__name__ + ".xmlstr")

class AwPluginRuleElement(object):

    def __init__(self, data, node):
        self.unique = not data.get("list", False)
        self.name = data["name"]
        self.plugins = self.__init_plugins(data["plugin"], node)

    def todict(self):
        return vars(self)

    def __init_plugins(self, plist, pnode):
        plugins = []
        ptree = pnode.tree()
        plist = plist if type(plist) is list else [plist]
        for pdata in plist:
            if type(pdata) is dict:
                plugins.extend(ptree.scan(pdata["scan"]))
            elif type(pdata) is str:
                if ptree.find(pdata):
                    plugins.append(pdata)
                else:
                    logger.warning("plugin is not found: {} in {}".format(pdata, pnode.path()))
            else:
                logger.warning("unknown plugin rule: {} in {}".format(pdata, pnode.path()))
        return plugins

class AwPluginFrameElement(object):

    def __init__(self, data):
        self.widget = data.get("widget", "node.frame")
        self.target = data.get("target")
        self.title  = data.get("title")

    def todict(self):
        return vars(self)

class AwPluginPanelElement(object):

    def __init__(self, data):
        self.widget = data.get("widget", "node.panel")
        self.frames = [AwPluginFrameElement(f) for f in data.get("frames", [])]

    def todict(self):
        return \
        {
            "widget": self.widget,
            "frames": [vars(data) for data in self.frames]
        }




if __name__ == "__main__":
    plugin = AwPluginTree()
    plugin.dump()