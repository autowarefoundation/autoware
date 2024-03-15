#!/usr/bin/env python3

# Copyright 2024 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
from tier4_system_msgs.msg import DiagnosticGraph


def print_table(lines: list, header: list):
    widths = [0 for _ in range(len(header))]
    lines.insert(0, header)
    for line in lines:
        widths = map(max, widths, map(len, line))
    widths = list(widths)
    lines.insert(0, ["-" * w for w in widths])
    lines.insert(2, ["-" * w for w in widths])
    for line in lines:
        line = map(lambda v, w: f"{v:{w}}", line, widths)
        line = " | ".join(line)
        print(f"| {line} |")


def get_level_text(level: int):
    if level == DiagnosticStatus.OK:
        return "OK"
    if level == DiagnosticStatus.WARN:
        return "WARN"
    if level == DiagnosticStatus.ERROR:
        return "ERROR"
    if level == DiagnosticStatus.STALE:
        return "STALE"
    return "-----"


class NodeData:
    def __init__(self, index, node):
        self.index = index
        self._node = node

    @property
    def level(self):
        return get_level_text(self._node.status.level)

    @property
    def name(self):
        return self._node.status.name

    @property
    def links(self):
        return " ".join(str(link.index) for link in self._node.links)

    @property
    def line(self):
        return [str(self.index), self.level, self.name, self.links]


class DumpNode(rclpy.node.Node):
    def __init__(self, args):
        super().__init__("dump_diagnostic_graph")
        self.sub = self.create_subscription(DiagnosticGraph, args.topic, self.callback, 1)

    def callback(self, msg):
        nodes = [NodeData(index, node) for index, node in enumerate(msg.nodes)]
        table = [node.line for node in nodes]
        print_table(table, ["index", "level", "name", "links"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/diagnostics_graph")
    args, unparsed = parser.parse_known_args()

    try:
        rclpy.init(args=unparsed)
        rclpy.spin(DumpNode(args))
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
