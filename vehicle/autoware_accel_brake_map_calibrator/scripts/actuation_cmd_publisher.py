#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Tier IV, Inc. All rights reserved.
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

from autoware_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float32Stamped
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class ActuationCmdPublisher(Node):
    def __init__(self):
        super().__init__("actuation_cmd_publisher")
        self.target_brake = 0.0
        self.target_accel = 0.0

        self.sub_acc = self.create_subscription(
            Float32Stamped, "/vehicle/tester/accel", self.on_accel, 1
        )
        self.sub_brk = self.create_subscription(
            Float32Stamped, "/vehicle/tester/brake", self.on_brake, 1
        )
        self.pub_actuation_cmd = self.create_publisher(
            ActuationCommandStamped, "/control/command/actuation_cmd", 1
        )
        self.pub_gear_cmd = self.create_publisher(GearCommand, "/control/command/gear_cmd", 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.on_timer)

    def on_brake(self, msg):
        self.target_brake = msg.data
        print(f"Set target brake : {self.target_brake}")

    def on_accel(self, msg):
        self.target_accel = msg.data
        print(f"Set target accel : {self.target_accel}")

    def on_timer(self):
        msg_actuation_cmd = ActuationCommandStamped()
        msg_actuation_cmd.actuation.steer_cmd = 0.0
        msg_actuation_cmd.header.stamp = self.get_clock().now().to_msg()
        msg_actuation_cmd.header.frame_id = "base_link"
        msg_actuation_cmd.actuation.accel_cmd = self.target_accel
        msg_actuation_cmd.actuation.brake_cmd = self.target_brake
        self.pub_actuation_cmd.publish(msg_actuation_cmd)

        msg_gear_cmd = GearCommand()
        msg_gear_cmd.stamp = self.get_clock().now().to_msg()
        msg_gear_cmd.command = GearCommand.DRIVE
        self.pub_gear_cmd.publish(msg_gear_cmd)

        print(
            f"publish ActuationCommand with accel: {self.target_accel}, brake: {self.target_brake}"
        )


def main(args=None):
    rclpy.init(args=args)

    actuation_cmd_publisher = ActuationCmdPublisher()

    rclpy.spin(actuation_cmd_publisher)

    actuation_cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
