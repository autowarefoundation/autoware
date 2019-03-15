import os
import yaml



class AwBaseTree(object):

    def __init__(self):
        self.nodes = {}

    def find(self, path):
        return self.nodes.get(path)

    def scan(self, path):
        return filter(lambda node: node.startswith(path), self.nodes.keys())

    def dump(self):
        for name in sorted(self.nodes.keys()):
            self.nodes[name].dump()


class AwBaseNode(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path

    def tree(self):
        return self.__tree

    def path(self):
        return self.__path

    def name(self):
        return os.path.basename(self.__path)
