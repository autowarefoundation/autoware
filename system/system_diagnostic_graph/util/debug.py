#!/usr/bin/env python3

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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from tier4_system_msgs.msg import DiagnosticGraph


class StructNode(Node):
    def __init__(self):
        super().__init__("system_diagnostic_graph_tools_struct")
        qos_struct = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_struct = self.create_subscription(
            DiagnosticGraph, "/diagnostics_graph/struct", self.callback, qos_struct
        )
        self.message = None

    def callback(self, msg):
        self.message = msg


class NodeSpinner:
    def __init__(self, time):
        self.time = time
        self.wait = True

    def on_timer(self):
        self.wait = False

    def spin(self, node):
        timer = node.create_timer(self.time, self.on_timer)
        while self.wait:
            rclpy.spin_once(node)
        node.destroy_timer(timer)


class GraphNode:
    def __init__(self, msg):
        self.name = msg.name
        self.links = msg.links


class GraphStruct:
    def __init__(self, msg):
        self.nodes = [GraphNode(node) for node in msg.nodes]

    @staticmethod
    def Subscribe():
        spin = NodeSpinner(1.0)
        node = StructNode()
        spin.spin(node)
        return GraphStruct(node.message)

    def visualize(self):
        from graphviz import Digraph

        graph = Digraph()
        graph.attr("node", shape="box")
        for node in self.nodes:
            graph.node(node.name)
            for link in node.links:
                graph.edge(node.name, self.nodes[link].name)
        graph.view()


if __name__ == "__main__":
    rclpy.init()
    graph = GraphStruct.Subscribe()
    rclpy.shutdown()
    graph.visualize()
