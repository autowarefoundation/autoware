#!/usr/bin/env python3

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

import sys
from threading import Thread

from autoware_adapi_version_msgs.srv import InterfaceVersion
import flask
import rclpy
from rclpy.node import Node

app = flask.Flask(__name__)
cli = None


@app.route("/interface/version")
def interface_version():
    req = InterfaceVersion.Request()
    res = cli.call(req)
    return flask.jsonify(convert_dict(res))


def create_service(node, service_type, service_name):
    cli = node.create_client(service_type, service_name)
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f"service not available, waiting again... ({service_name}")
    return cli


def convert_dict(msg):
    fields = getattr(msg, "get_fields_and_field_types", None)
    if not fields:
        return msg
    return {field: convert_dict(getattr(msg, field)) for field in fields()}


def spin_ros_node():
    global cli
    node = Node("ad_api_default_web_server")
    cli = create_service(node, InterfaceVersion, "/api/interface/version")
    rclpy.spin(node)


if __name__ == "__main__":
    rclpy.init(args=sys.argv, signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    thread = Thread(target=spin_ros_node)
    thread.start()
    app.run(host="localhost", port=8888, debug=False)
    rclpy.shutdown()
    thread.join()
