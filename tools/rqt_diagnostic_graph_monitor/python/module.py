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


from rclpy.node import Node
from tier4_system_msgs.msg import DiagGraphStatus
from tier4_system_msgs.msg import DiagGraphStruct

from .graph import Graph
from .utils import default_qos
from .utils import durable_qos
from .utils import foreach


class MonitorModule:
    def __init__(self, node: Node):
        self.graph = None
        self.struct_callbacks = []
        self.status_callbacks = []
        self.node = node
        self.sub_struct = self.subscribe_struct()
        self.sub_status = self.subscribe_status()

    def append_struct_callback(self, callback):
        self.struct_callbacks.append(callback)

    def append_status_callback(self, callback):
        self.status_callbacks.append(callback)

    def on_struct(self, msg):
        self.graph = Graph(msg)
        foreach(self.struct_callbacks, lambda callback: callback(self.graph))

    def on_status(self, msg):
        self.graph.update(msg)
        foreach(self.status_callbacks, lambda callback: callback(self.graph))

    def subscribe_struct(self):
        return self.node.create_subscription(
            DiagGraphStruct, "/diagnostics_graph/struct", self.on_struct, durable_qos(1)
        )

    def subscribe_status(self):
        return self.node.create_subscription(
            DiagGraphStatus, "/diagnostics_graph/status", self.on_status, default_qos(1)
        )

    def shutdown(self):
        self.node.destroy_subscription(self.sub_struct)
        self.node.destroy_subscription(self.sub_status)
