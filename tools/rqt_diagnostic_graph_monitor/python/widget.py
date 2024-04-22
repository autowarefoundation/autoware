# Copyright 2023 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from .graph import Graph
from .items import MonitorItem
from .utils import foreach


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self):
        super().__init__()
        self.graph = None
        self.items = []
        self.root_widget = QtWidgets.QTreeWidget()
        self.tree_widget = QtWidgets.QTreeWidget()
        self.root_widget.setHeaderLabels(["Top-level"])
        self.tree_widget.setHeaderLabels(["Subtrees"])
        self.addWidget(self.root_widget)
        self.addWidget(self.tree_widget)

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(500)

    def shutdown(self):
        self.clear_graph()

    def on_timer(self):
        foreach(self.items, lambda item: item.update())

    def on_graph(self, graph: Graph):
        self.clear_graph()
        self.build_graph(graph)

    def clear_graph(self):
        self.graph = None
        self.items = []
        self.root_widget.clear()
        self.tree_widget.clear()

    def build_graph(self, graph: Graph):
        self.graph = graph
        root_units = filter(self.is_root_unit, self.graph.units)
        tree_units = filter(self.is_tree_unit, self.graph.units)
        root_items = [MonitorItem(None, unit) for unit in root_units]
        tree_items = [MonitorItem(None, unit) for unit in tree_units]
        link_items = [MonitorItem(link, link.child) for link in self.graph.links]
        self.items = root_items + tree_items + link_items

        # Note: overwrite link items with root/tree items if there is more than one.
        parents = {}
        for items in [link_items, tree_items, root_items]:
            parents.update({item.unit: item.item for item in items})

        # Connect tree widget items.
        root_widget_item = self.root_widget.invisibleRootItem()
        tree_widget_item = self.tree_widget.invisibleRootItem()
        for item in root_items:
            root_widget_item.addChild(item.item)
        for item in tree_items:
            tree_widget_item.addChild(item.item)
        for item in link_items:
            parents[item.link.parent].addChild(item.item)

    @staticmethod
    def is_root_unit(unit):
        return len(unit.parents) == 0

    @staticmethod
    def is_tree_unit(unit):
        return len(unit.parents) >= 2 and len(unit.children) != 0
