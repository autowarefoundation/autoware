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


from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_adapi_v1_msgs.msg import RouteState
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from tier4_system_msgs.msg import ModeChangeAvailable

modules = [
    "control",
    "localization",
    "map",
    "perception",
    "planning",
    "sensing",
    "system",
    "vehicle",
]


class Sub(object):
    def __init__(self, node, topic_type, topic_name, durable):
        qos = self.durable_qos(1) if durable else self.default_qos(1)
        self.sub = node.create_subscription(topic_type, topic_name, self.callback, qos)
        self.msg = None

    def callback(self, msg):
        self.msg = msg

    @staticmethod
    def durable_qos(depth):
        r = QoSReliabilityPolicy.RELIABLE
        d = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return QoSProfile(depth=depth, reliability=r, durability=d)

    @staticmethod
    def default_qos(depth):
        r = QoSReliabilityPolicy.RELIABLE
        d = QoSDurabilityPolicy.VOLATILE
        return QoSProfile(depth=depth, reliability=r, durability=d)


class GuideNode(Node):
    def __init__(self):
        # fmt: off
        super().__init__("guide")
        self.localization = Sub(self, LocalizationInitializationState, "/api/localization/initialization_state", True)
        self.routing = Sub(self, RouteState, "/api/routing/state", True)
        self.operation_mode = Sub(self, OperationModeState, "/api/operation_mode/state", True)
        self.launch_states = [Sub(self, ModeChangeAvailable, f"/system/component_state_monitor/component/launch/{module}", True) for module in modules]
        self.auto_states = [Sub(self, ModeChangeAvailable, f"/system/component_state_monitor/component/autonomous/{module}", True) for module in modules]
        self.timer = self.create_timer(1.0, self.check_callback)
        self.previous_message = None
        # fmt: on

    def check_callback(self):
        message = self.check_localization()
        if self.previous_message != message:
            print(message)
            self.previous_message = message

    def check_localization(self):
        if self.localization.msg is None:
            return "The localization state is not received. Please check ADAPI and localization component works correctly."
        elif self.localization.msg.state == LocalizationInitializationState.INITIALIZED:
            return self.check_routing()
        else:
            return "The vehicle pose is not estimated. Please set an initial pose or check GNSS."

    def check_routing(self):
        if self.routing.msg is None:
            return "The routing state is not received. Please check ADAPI and planning component works correctly."
        elif self.routing.msg.state == RouteState.UNSET:
            return "The route is not set. Please set a goal pose."
        elif self.routing.msg.state == RouteState.SET:
            return self.check_operation_mode()
        elif self.routing.msg.state == RouteState.ARRIVED:
            return "The vehicle has reached the goal of the route. Please reset a route."

    def check_operation_mode(self):
        if self.operation_mode.msg is None:
            return "The operation mode state is not received. Please check ADAPI and control component works correctly."
        elif self.operation_mode.msg.mode == OperationModeState.AUTONOMOUS:
            return "The vehicle is driving autonomously."
        elif self.operation_mode.msg.mode == OperationModeState.LOCAL:
            return "The vehicle is driving by a local operator."
        elif self.operation_mode.msg.mode == OperationModeState.REMOTE:
            return "The vehicle is driving by a remote operator."
        elif self.operation_mode.msg.is_autonomous_mode_available:
            return "The vehicle is ready. Please change the operation mode to autonomous."
        else:
            return self.check_autonomous_condition()

    def check_autonomous_condition(self):
        states = [state.msg.available if state.msg else False for state in self.auto_states]
        if all(states):
            return "Unable to safely switch to automatic mode. Please stop or satisfy the driving mode change condition."
        else:
            components = ",".join([module for module, state in zip(modules, states) if not state])
            return f"The topic rate error is detected. Please check [{components}] components."


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(GuideNode())
    rclpy.shutdown()
