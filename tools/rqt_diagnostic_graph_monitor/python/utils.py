# Copyright 2024 The Autoware Contributors
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

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile


def default_qos(depth):
    return QoSProfile(depth=depth)


def durable_qos(depth):
    return QoSProfile(depth=depth, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def foreach(iterable, function):
    for item in iterable:
        function(item)
