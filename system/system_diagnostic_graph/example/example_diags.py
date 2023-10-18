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

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
import rclpy.qos


class DummyDiagnostics(rclpy.node.Node):
    def __init__(self, names):
        super().__init__("dummy_diagnostics")
        qos = rclpy.qos.qos_profile_system_default
        self.diags = self.create_publisher(DiagnosticArray, "/diagnostics", qos)
        self.timer = self.create_timer(0.5, self.on_timer)
        self.count = 0
        self.array = [self.create_status(name) for name in names]

    def on_timer(self):
        error_index = int(self.count / 10)
        for index, status in enumerate(self.array, 1):
            if index == error_index:
                status.level = DiagnosticStatus.ERROR
                status.message = "ERROR"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "OK"

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = self.array
        self.count = (self.count + 1) % 60
        self.diags.publish(diagnostics)

    @staticmethod
    def create_status(name: str):
        return DiagnosticStatus(name=name, hardware_id="example")


if __name__ == "__main__":
    diags = [
        "lidar_driver/top: status",
        "lidar_driver/front: status",
        "radar_driver/front: status",
        "external_command_checker: joystick_command",
        "external_command_checker: remote_command",
    ]
    rclpy.init()
    rclpy.spin(DummyDiagnostics(diags))
    rclpy.shutdown()
