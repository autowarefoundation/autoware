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

from autoware_adapi_v1_msgs.msg import LocalizationInitializationState as LocalizationState
from autoware_adapi_v1_msgs.msg import RouteState
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
import rclpy.node
import rclpy.qos


class ComponentStateDiagnostics(rclpy.node.Node):
    def __init__(self):
        super().__init__("component_state_diagnostics")
        durable_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )

        self.timer = self.create_timer(0.5, self.on_timer)
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.sub1 = self.create_subscription(
            LocalizationState,
            "/api/localization/initialization_state",
            self.on_localization,
            durable_qos,
        )
        self.sub2 = self.create_subscription(
            RouteState, "/api/routing/state", self.on_routing, durable_qos
        )

        self.diags = DiagnosticArray()
        self.diags.status.append(
            DiagnosticStatus(
                level=DiagnosticStatus.STALE, name="component_state_diagnostics: localization_state"
            )
        )
        self.diags.status.append(
            DiagnosticStatus(
                level=DiagnosticStatus.STALE, name="component_state_diagnostics: route_state"
            )
        )

    def on_timer(self):
        self.diags.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.diags)

    def on_localization(self, msg):
        self.diags.status[0].level = (
            DiagnosticStatus.OK
            if msg.state == LocalizationState.INITIALIZED
            else DiagnosticStatus.ERROR
        )

    def on_routing(self, msg):
        self.diags.status[1].level = (
            DiagnosticStatus.OK if msg.state != RouteState.UNSET else DiagnosticStatus.ERROR
        )


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(ComponentStateDiagnostics())
    rclpy.shutdown()
