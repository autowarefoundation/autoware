#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import Engage
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

accel_topic = "/localization/acceleration"
odom_topic = "/localization/kinematic_state"
in_gate_cmd_topic = "/control/trajectory_follower/control_cmd"
out_gate_cmd_topic = "/control/command/control_cmd"
engage_topic = "/autoware/engage"


# This node measure the delays between when we want ego to move, and when it actually starts moving.
# There are 2 cases measured:
# 1) After engaging
#   Before autoware is engaged, assuming a valid and non stopping trajectory is generated
#   by the planning module,
#   then the controller should output a control command with positive speed and acceleration.
#   In that case, the delay measured is the duration between autoware being engaged,
#   and the vehicle starting to move (based on the measured velocity).
# 2) Restart
#   When already engaged, the ego vehicle may stop and restart multiple times.
#   (e.g., when stopping at a red light). In that case,
#   the trajectory is stopping and the control command has a non-positive velocity and acceleration.
#   In that case, the delay measured is the duration between the controller's command switching to
#   a positive velocity and acceleration, and the ego vehicle actually starting to move.
class DelaysChecker(Node):
    def __init__(self):
        super().__init__("delays_checker")

        self.autoware_engage = None
        self.ego_is_stopped = True
        self.prev_in_cmd = None
        self.last_engage_time = None
        self.last_start_time = None
        self.current_accel = 0.0

        # planning path and trajectories
        self.sub_accel = self.create_subscription(
            AccelWithCovarianceStamped,
            accel_topic,
            self.CallBackAccel,
            1,
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            odom_topic,
            self.CallBackOdom,
            1,
        )
        self.sub_engage = self.create_subscription(Engage, engage_topic, self.CallBackEngage, 1)
        self.sub_in_gate_cmd = self.create_subscription(
            AckermannControlCommand,
            in_gate_cmd_topic,
            self.CallBackInCmd,
            1,
        )
        self.sub_out_gate_cmd = self.create_subscription(
            AckermannControlCommand,
            out_gate_cmd_topic,
            self.CallBackOutCmd,
            1,
        )

    def CallBackEngage(self, msg):
        if not self.autoware_engage or msg.engage != self.autoware_engage:
            self.autoware_engage = msg.engage
            if self.autoware_engage == 1:
                self.last_engage_time = self.get_clock().now()

    def CallBackInCmd(self, msg):
        is_start = (
            self.prev_in_cmd
            and self.ego_is_stopped
            and self.prev_in_cmd.longitudinal.acceleration < 0.0
            and msg.longitudinal.acceleration > 0.0
        )
        if is_start:
            self.last_start_time = self.get_clock().now()
        self.prev_in_cmd = msg

    def CallBackOutCmd(self, msg):
        None

    def CallBackAccel(self, msg):
        self.current_accel = msg.accel.accel.linear.x

    def CallBackOdom(self, msg):
        if msg.twist.twist.linear.x < 1e-6:
            if not self.ego_is_stopped:
                self.last_engage_time = None
            self.ego_is_stopped = True
        elif self.ego_is_stopped:
            # Ego starts to move
            now = self.get_clock().now()
            delay_ms = (
                (now - self.last_engage_time).nanoseconds * 1e-6
                if self.last_engage_time
                else (
                    (now - self.last_start_time).nanoseconds * 1e-6 if self.last_start_time else 0.0
                )
            )
            self.get_logger().info(
                "Move delay {}ms ({})".format(
                    delay_ms, ("after engage" if self.last_engage_time else "restart")
                )
            )
            self.ego_is_stopped = False


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DelaysChecker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
