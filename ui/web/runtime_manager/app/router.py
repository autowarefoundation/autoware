#!/usr/bin/env python
# coding: utf-8

from flask import Flask, send_from_directory, current_app, render_template, Response, jsonify
from flask_cors import CORS
from os.path import abspath, dirname
from config.env import env
from controllers.ros_controller import ROSController
import traceback

from pprint import PrettyPrinter
pp = PrettyPrinter(indent=2).pprint


rosController = ROSController(env)

# flask = Flask(__name__, instance_path=dirname(__file__)+"/secret")
flask = Flask(__name__)
CORS(flask)
# with flask.app_context():


def api_response(code=200, message={}):
    response = jsonify(message)
    response.status_code = code
    return response


@flask.route('/', methods=["GET"])
def root():
    print("root")
    rosController.killall()
    return send_from_directory(
        directory="./views/", filename="index.html")


@flask.route("/.config/model/<path:path>", methods=["GET"])
def getVehicleModel(path):
    print("getVehicleModel", path)
    return send_from_directory(
        directory='../../../ros/src/.config/model/', filename=path, as_attachment=True)


@flask.route("/res/<type>/<path:path>", methods=["GET"])
def getResources(type, path):
    print("getResources", type, path)
    if type in ["lib", "node_modules", "build", "static"]:
        return send_from_directory(
            directory='./views/'+type+"/", filename=path, as_attachment=True)
    else:
        return api_response(500, {"type": type, "path": path})


@flask.route('/roslaunch/<target>/<mode>')
def roslaunch(target, mode):
    print("roslaunch", target, mode)

    if target == "rosbag-play":
        if mode == "on":
            rosController.play_rosbag()
        else:
            rosController.pause_rosbag()
        return api_response(200, {"target": target, "mode": mode})

    try:
        rosController.launch(mode, target)
        return api_response(200, {"target": target, "mode": mode})
    except:
        traceback.print_exc()
        return api_response(500, {"target": target, "mode": mode})


@flask.route("/rosbag/<cat>", methods=["GET"])
def rosbag(cat):
    if cat == "load":
        rosController.load_rosbag()
        response = api_response(200, {"rosbag": cat})
    elif cat == "play":
        if rosController.get_rosbag_state() is "stop":
            rosController.load_rosbag()
            rosController.setup_viewer()
        rosController.play_rosbag()
        response = api_response(200, {"rosbag": cat})
    elif cat == "pause":
        rosController.pause_rosbag()
        response = api_response(200, {"rosbag": cat})
    elif cat == "stop":
        rosController.stop_rosbag()
        response = api_response(200, {"rosbag": cat})
    else:
        response = api_response(500, {"error": cat})
    return response


if __name__ == '__main__':
    print("flask run")
    flask.run(host=env["AUTOWARE_WEB_UI_HOST"], port=env["AUTOWARE_WEB_UI_PORT"])
    # flask.run(debug=True)
