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


import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from tier4_debug_msgs.msg import Float64Stamped


class ProcessingTimeSubscriber(Node):
    def __init__(self, max_display_time=150, display_frequency=5.0):
        super().__init__("processing_time_subscriber")
        self.data_map = {}  # {topic_name: data_value}
        self.max_display_time = max_display_time
        self.get_topic_list()

        # Timer to print data at given frequency
        self.create_timer(1.0 / display_frequency, self.print_data)

    def get_topic_list(self):
        # Get list of existing topics in the system
        topic_list = self.get_topic_names_and_types()

        # Subscribe to topics with 'processing_time_ms' suffix
        for topic, types in topic_list:
            if topic.endswith("processing_time_ms"):
                self.create_subscription(
                    Float64Stamped, topic, lambda msg, topic=topic: self.callback(msg, topic), 1
                )
                self.get_logger().info(f"Subscribed to {topic} | Type: {types}")

    def callback(self, msg, topic):
        self.data_map[topic] = msg.data

    def print_data(self):
        # Clear terminal
        os.system("cls" if os.name == "nt" else "clear")

        if not self.data_map:
            print("No topics available.")
            return

        # Get the maximum topic name length for alignment
        max_topic_length = max(map(len, self.data_map.keys()))

        # Generate the header based on the max_display_time
        header_str = "topics".ljust(max_topic_length) + ":"
        for i in range(0, self.max_display_time + 1, 20):
            header_str += f" {i}ms".ljust(20)

        # Print the header
        print(header_str)
        print("-" * len(header_str))

        # Print each topic's data
        for topic, data in self.data_map.items():
            # Round the data to the third decimal place for display
            data_rounded = round(data, 3)
            # Calculate the number of bars to be displayed (clamped to max_display_time)
            num_bars = int(min(data, self.max_display_time - 1)) + 1
            print(f"{topic.ljust(max_topic_length)}: {'|' * num_bars} ({data_rounded}ms)")


def main(args=None):
    # Get the command line arguments before argparse processes them
    cmd_args = sys.argv[1:]

    parser = argparse.ArgumentParser(description="Processing Time Subscriber Parameters")
    parser.add_argument(
        "-m", "--max_display_time", type=int, default=150, help="Maximum display time in ms"
    )
    parser.add_argument(
        "-f", "--display_frequency", type=float, default=5.0, help="Display frequency in Hz"
    )
    args = parser.parse_args()

    rclpy.init(args=cmd_args)  # Use the original command line arguments here
    subscriber = ProcessingTimeSubscriber(
        max_display_time=args.max_display_time, display_frequency=args.display_frequency
    )
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
