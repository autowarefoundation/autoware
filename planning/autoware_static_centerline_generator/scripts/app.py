#!/usr/bin/env python3

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

import json
import uuid

from autoware_static_centerline_generator.srv import LoadMap
from autoware_static_centerline_generator.srv import PlanPath
from autoware_static_centerline_generator.srv import PlanRoute
from flask import Flask
from flask import jsonify
from flask import request
from flask_cors import CORS
import rclpy
from rclpy.node import Node

rclpy.init()
node = Node("static_centerline_generator_http_server")

app = Flask(__name__)
CORS(app)


def create_client(service_type, server_name):
    # create client
    cli = node.create_client(service_type, server_name)
    while not cli.wait_for_service(timeout_sec=1.0):
        print("{} service not available, waiting again...".format(server_name))
    return cli


@app.route("/map", methods=["POST"])
def get_map():
    data = request.get_json()

    map_uuid = str(uuid.uuid4())
    global map_id
    map_id = map_uuid

    # create client
    cli = create_client(LoadMap, "/planning/autoware_static_centerline_generator/load_map")

    # request map loading
    req = LoadMap.Request(map=data["map"])
    future = cli.call_async(req)

    # get result
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # error handling
    if res.message == "InvalidMapFormat":
        return jsonify(code=res.message, message="Map format is invalid."), 400
    elif res.message != "":
        return (
            jsonify(
                code="InternalServerError",
                message="Error occurred on the server. Please check the terminal.",
            ),
            500,
        )

    return map_uuid


@app.route("/planned_route", methods=["GET"])
def post_planned_route():
    args = request.args.to_dict()
    global map_id
    if map_id != args.get("map_id"):
        # TODO(murooka) error handling for map_id mismatch
        print("map_id is not correct.")

    # create client
    cli = create_client(PlanRoute, "/planning/autoware_static_centerline_generator/plan_route")

    # request route planning
    req = PlanRoute.Request(
        start_lane_id=int(args.get("start_lane_id")), end_lane_id=int(args.get("end_lane_id"))
    )
    future = cli.call_async(req)

    # get result
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # error handling
    if res.message == "MapNotFound":
        return jsonify(code=res.message, message="Map is missing."), 404
    elif res.message == "RouteNotFound":
        return jsonify(code=res.message, message="Planning route failed."), 404
    elif res.message != "":
        return (
            jsonify(
                code="InternalServerError",
                message="Error occurred on the server. Please check the terminal.",
            ),
            500,
        )

    return json.dumps(tuple(res.lane_ids))


@app.route("/planned_path", methods=["GET"])
def post_planned_path():
    args = request.args.to_dict()
    global map_id
    if map_id != args.get("map_id"):
        # TODO(murooka) error handling for map_id mismatch
        print("map_id is not correct.")

    # create client
    cli = create_client(PlanPath, "/planning/autoware_static_centerline_generator/plan_path")

    # request path planning
    route_lane_ids = [eval(i) for i in request.args.getlist("route[]")]
    req = PlanPath.Request(route=route_lane_ids)
    future = cli.call_async(req)

    # get result
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    # error handling
    if res.message == "MapNotFound":
        return jsonify(code=res.message, message="Map is missing."), 404
    elif res.message == "LaneletsNotConnected":
        return (
            jsonify(
                code=res.message,
                message="Lanelets are not connected.",
                object_ids=tuple(res.unconnected_lane_ids),
            ),
            400,
        )
    elif res.message != "":
        return (
            jsonify(
                code="InternalServerError",
                message="Error occurred on the server. Please check the terminal.",
            ),
            500,
        )

    # create output json
    result_json = []
    for points_with_lane_id in res.points_with_lane_ids:
        current_lane_points = []
        for geom_point in points_with_lane_id.points:
            point = {"x": geom_point.x, "y": geom_point.y, "z": geom_point.z}
            current_lane_points.append(point)

        current_result_json = {}
        current_result_json["lane_id"] = int(points_with_lane_id.lane_id)
        current_result_json["points"] = current_lane_points

        result_json.append(current_result_json)

    return json.dumps(tuple(result_json))


if __name__ == "__main__":
    app.debug = True
    app.secret_key = "tmp_secret_key"
    app.run(host="localhost", port=4010)
