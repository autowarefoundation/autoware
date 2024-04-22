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
import rclpy.time


class ComponentStateDiagnostics:
    def __init__(self, node, diag_name, topic_name, topic_type, topic_qos, diag_func):
        self.node = node
        self.name = diag_name
        self.func = diag_func
        self.sub = node.create_subscription(topic_type, topic_name, self.callback, topic_qos)
        self.level = DiagnosticStatus.STALE
        self.stamp = None

    def callback(self, msg):
        self.level = DiagnosticStatus.OK if self.func(msg) else DiagnosticStatus.ERROR
        self.stamp = self.node.get_clock().now()

    def update(self, stamp):
        if self.stamp:
            elapsed = (stamp - self.stamp).nanoseconds * 1e-9
            if 3.0 < elapsed:
                self.level = DiagnosticStatus.STALE
                self.stamp = None

    def message(self):
        status = DiagnosticStatus()
        status.name = self.name
        status.level = self.level
        return status


class ComponentStateDiagnosticsNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("component_state_diagnostics")
        durable_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )

        self.timer = self.create_timer(0.5, self.on_timer)
        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 1)
        self.states = []
        self.states.append(
            ComponentStateDiagnostics(
                self,
                "component_state_diagnostics: localization_state",
                "/api/localization/initialization_state",
                LocalizationState,
                durable_qos,
                lambda msg: msg.state == LocalizationState.INITIALIZED,
            )
        )
        self.states.append(
            ComponentStateDiagnostics(
                self,
                "component_state_diagnostics: route_state",
                "/api/routing/state",
                RouteState,
                durable_qos,
                lambda msg: msg.state != RouteState.UNSET,
            )
        )

    def on_timer(self):
        stamp = self.get_clock().now()
        array = DiagnosticArray()
        array.header.stamp = stamp.to_msg()
        for state in self.states:
            array.status.append(state.message())
        self.pub.publish(array)


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(ComponentStateDiagnosticsNode())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
