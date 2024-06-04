#! /usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
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


from autoware_vehicle_msgs.msg import VelocityReport
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from tier4_vehicle_msgs.msg import ActuationStatusStamped


class ActuationStatusPublisher(Node):
    def __init__(self):
        super().__init__("actuation_status_publisher")

        qos_profile = QoSProfile(depth=1)
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.pub = self.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", qos_profile
        )
        self.sub = self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self.callback, qos_profile
        )

    def callback(self, msg):
        data = ActuationStatusStamped()
        data.header = msg.header
        data.status.accel_status = msg.longitudinal_velocity * 0.1
        data.status.brake_status = 0.1
        data.status.steer_status = 0.1
        self.pub.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = ActuationStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
