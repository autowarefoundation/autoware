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
            if env["AUTOWARE_WEB_UI_HOST"] == "localhost":
                if connection.laddr[1] == int(env["AUTOWARE_WEB_UI_PORT"]):
                    pids[proc["pid"]] = psutil.Process(pid=proc["pid"]);
            else:
                if connection.raddr == (socket.gethostbyname(env["AUTOWARE_WEB_UI_HOST"]), int(env["AUTOWARE_WEB_UI_PORT"])):
                    pids[proc["pid"]] = psutil.Process(pid=proc["pid"]);

    if 0 < len(pids):
        print(pids)

    if len(pids.keys()) == 1:
        pids.values()[0].kill()
        pids.values()[0].wait()


def kill_ros():
    call(["pkill", "-f", "ros"]);
    call(["pkill", "-f", "rosbag"]);


if __name__ == '__main__':
    print("kill connection")
    kill_connection()
    kill_ros()
