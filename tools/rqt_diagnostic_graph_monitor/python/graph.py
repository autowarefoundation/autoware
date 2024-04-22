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

from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.time import Time


class DummyStatus:
    def __init__(self):
        self.level = DiagnosticStatus.STALE


class BaseUnit:
    def __init__(self, status=DummyStatus()):
        self._parents = []
        self._children = []
        self._path = None
        self._type = None
        self._status = status

    @property
    def parents(self):
        return self._parents

    @property
    def children(self):
        return self._children

    @property
    def path(self):
        return self._path

    @property
    def kind(self):
        return self._type


class NodeUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__()
        self._path = struct.path
        self._type = struct.type

    def update(self, status):
        self._status = status

    @property
    def level(self):
        return self._status.level


class DiagUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__()
        self._path = struct.path
        self._name = struct.name
        self._type = "diag"

    def update(self, status):
        self._status = status

    @property
    def level(self):
        return self._status.level


class UnitLink:
    def __init__(self, parent: BaseUnit, child: BaseUnit):
        self._parent = parent
        self._child = child
        parent._children.append(self)
        child._parents.append(self)

    def update(self, status):
        self.status = status

    @property
    def parent(self):
        return self._parent

    @property
    def child(self):
        return self._child


class Graph:
    def __init__(self, msg):
        self._struct_stamp = Time.from_msg(msg.stamp)
        self._status_stamp = None
        self._id = msg.id
        self._nodes = [NodeUnit(struct) for struct in msg.nodes]
        self._diags = [DiagUnit(struct) for struct in msg.diags]
        self._units = self._nodes + self._diags
        self._links = []
        for struct in msg.links:
            units = self._diags if struct.is_leaf else self._nodes
            nodes = self._nodes
            self._links.append(UnitLink(nodes[struct.parent], units[struct.child]))

    def update(self, msg):
        if msg.id == self._id:
            self._status_stamp = Time.from_msg(msg.stamp)
            for node, status in zip(self._nodes, msg.nodes):
                node.update(status)
            for diag, status in zip(self._diags, msg.diags):
                diag.update(status)
            for link, status in zip(self._links, msg.links):
                link.update(status)

    @property
    def units(self):
        return self._units

    @property
    def links(self):
        return self._links
