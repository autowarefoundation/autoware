#!/usr/bin/env python
# coding: utf-8

import psutil
import socket
from subprocess import call, Popen
from config.env import env


def kill_connection():
    procs = list(filter(
        lambda x: x["name"] == "python" and 0 < len(x["connections"]),
        map(
            lambda x: x.as_dict(attrs=["name", "pid", "connections"]),
            psutil.process_iter())))

    pids = {}
    for proc in procs:
        for connection in proc["connections"]:
            if connection.laddr == (
            socket.gethostbyname(env["AUTOWARE_WEB_UI_HOST"]), int(env["AUTOWARE_WEB_UI_PORT"])):
                pids[proc["pid"]] = psutil.Process(pid=proc["pid"]);

    if 0 < len(pids):
        for pid in pids.values():
            print(pid)
            pid.kill()
            pid.wait()


def kill_ros():
    call(["pkill", "-f", "ros"]);
    call(["pkill", "-f", "rosbag"]);


def kill_web_video_server():
    call(["pkill", "-f", "web_video_server"]);


if __name__ == '__main__':
    print("kill connection")
    kill_connection()
    kill_ros()
