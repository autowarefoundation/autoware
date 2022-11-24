# Copyright 2022 TIER IV, Inc.
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

from autoware_adapi_version_msgs.srv import InterfaceVersion
import rclpy
import rclpy.node

rclpy.init()
node = rclpy.node.Node("test_client")

client = node.create_client(InterfaceVersion, "/api/interface/version")
if not client.wait_for_service(timeout_sec=3.0):
    exit(1)

request = InterfaceVersion.Request()
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()

if response.major != 1:
    exit(1)
if response.minor != 0:
    exit(1)
if response.patch != 0:
    exit(1)

rclpy.shutdown()
