from ..core import myutils


class AwLaunchTreeMirror(object):
    
    def __init__(self, client):
        self.nodes = {}
        self.cache = {}
        self.client = client

    def clear(self, lpath = None):
        if lpath:
            self.cache.pop(lpath, None)
        else:
            self.cache.clear()

    def find(self, path):
        if path not in self.cache:
            self.cache[path] = self.client.find_node(path)
        return self.cache[path]

    def create(self, path):
        if path not in self.nodes:
            self.nodes[path] = AwLaunchNodeMirror(self, path)
        return self.nodes[path]

    def remove(self, path, node):
        self.nodes.pop(path)



class AwLaunchNodeMirror(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__refs = []

    def __find(self):
        return self.__tree.find(self.__path)

    def tostring(self):
        return self.__find().tostring()

    def status(self):
        node = self.__find()
        if node.status == node.STOP: return "stop"
        if node.status == node.EXEC: return "exec"
        if node.status == node.TERM: return "term"
        return "exec/term"

    def isleaf(self):
        return self.__find().plugin.isleaf()

    def path(self):
        return self.__find().nodepath()

    def name(self):
        return self.__find().nodename()
    
    def plugin(self):
        return self.__find().plugin

    def config(self):
        #return self.__find().config
        return self.__find().config.copy()

    def update(self, ldata):
        return self.__tree.client.update_node(self.__path, ldata)

    def launch(self, mode):
        self.__tree.client.launch_node(self.__path, mode)

    def listnode(self, this):
        return map(lambda node: node.nodepath(), self.__find().listnode(this))

    def haschild(self, name):
        return self.__find().haschild(name)

    def getchild(self, name):
        return self.__tree.create(self.__path + "/" + name)

    def addchild(self, lname, ppath):
        return self.__tree.client.create_node(self.__path + "/" + lname, ppath)

    def children(self):
        mirrored_children = []
        for child in self.__find().children():
            mirrored_children.append(self.__tree.create(child.nodepath()))
        return mirrored_children

    def childnames(self):
        return map(lambda node: node.nodename(), self.__find().children())

    def childnodes(self):
        return [self.__tree.create(child.nodepath()) for child in self.__find().children()]

    def get_config(self, key, value = None):
        return self.__find().config.get(key, value)

    def generate_launch(self):
        return self.__find().generate_launch()




#class AwLaunchNodeMirror2(object):
#
#    def __init__(self, writer):
#        self.writer = writer
#    
#    def __getattr__(self, name):
#        return getattr(self.writer, name)
