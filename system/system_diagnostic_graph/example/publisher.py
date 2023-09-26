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

import argparse

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
import rclpy.qos


class DummyDiagnostics(rclpy.node.Node):
    def __init__(self, array):
        super().__init__("dummy_diagnostics")
        qos = rclpy.qos.qos_profile_system_default
        self.diags = self.create_publisher(DiagnosticArray, "/diagnostics", qos)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.array = [self.create_status(*data) for data in array]

    def on_timer(self):
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = self.array
        self.diags.publish(diagnostics)

    @staticmethod
    def create_status(name: str, level: int):
        return DiagnosticStatus(level=level, name=name, message="OK", hardware_id="test")


if __name__ == "__main__":
    data = {
        "ok": [
            ("/sensing/lidars/top", DiagnosticStatus.OK),
            ("/sensing/lidars/front", DiagnosticStatus.OK),
            ("/sensing/radars/front", DiagnosticStatus.OK),
            ("/planning/route", DiagnosticStatus.OK),
            ("/external/remote_command", DiagnosticStatus.OK),
        ],
        "front-lidar": [
            ("/sensing/lidars/top", DiagnosticStatus.OK),
            ("/sensing/lidars/front", DiagnosticStatus.ERROR),
            ("/sensing/radars/front", DiagnosticStatus.OK),
            ("/planning/route", DiagnosticStatus.OK),
            ("/external/remote_command", DiagnosticStatus.OK),
        ],
        "front-radar": [
            ("/sensing/lidars/top", DiagnosticStatus.OK),
            ("/sensing/lidars/front", DiagnosticStatus.OK),
            ("/sensing/radars/front", DiagnosticStatus.ERROR),
            ("/planning/route", DiagnosticStatus.OK),
            ("/external/remote_command", DiagnosticStatus.OK),
        ],
        "front": [
            ("/sensing/lidars/top", DiagnosticStatus.OK),
            ("/sensing/lidars/front", DiagnosticStatus.ERROR),
            ("/sensing/radars/front", DiagnosticStatus.ERROR),
            ("/planning/route", DiagnosticStatus.OK),
            ("/external/remote_command", DiagnosticStatus.OK),
        ],
    }

    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="ok")
    args = parser.parse_args()

    rclpy.init()
    rclpy.spin(DummyDiagnostics(data[args.data]))
    rclpy.shutdown()
