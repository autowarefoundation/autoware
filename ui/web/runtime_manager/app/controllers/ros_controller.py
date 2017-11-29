#!/usr/bin/env python
# coding: utf-8

import rospy
import roslaunch

from subprocess import call, Popen
from os.path import abspath, dirname, realpath
from os import listdir
from time import sleep

from rosbag_controller import ROSBAGController


class ROSController(object):
    def __init__(self, env):
        self.__pids = []
        self.__env = env
        self.__devnull = open("/dev/null", 'w')
        self.__results = {}
        self.__popens = {}
        self.__rosbag = ROSBAGController()

        self.__path = abspath(dirname(__file__))

        self.__launches = {}
        self.__uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.__uuid)

    def __del__(self):
        self.__devnull.close()

    def launch(self, domain="map", target="map", mode="on"):
        launch_id = "/".join([domain, target])
        if mode == "on":
            self.__launches[launch_id] = roslaunch.parent.ROSLaunchParent(
                self.__uuid, [self.__path + "/res/{}/{}.launch".format(domain, target)])
            self.__launches[launch_id].start()
        else:
            if launch_id in self.__launches:
                self.__launches[launch_id].shutdown()
        return True

    def killall(self):
        for launch in self.__launches.values():
            launch.shutdown()
        call(["pkill", "-f", "ros"]);
        call(["pkill", "-f", "rosbag"]);
        return True

    def get_rosbag_state(self):
        current_prev, duration = self.__rosbag.get_progress()
        if duration is None:
            return "stop"
        else:
            current, duration = self.__rosbag.get_progress()
            if current == current_prev:
                return "pause"
            else:
                return "play"

    def load_rosbag(self):
        self.__rosbag.load(self.__env["PATH_ROSBAG_FILE"])
        sleep(1)
        self.__rosbag.play()
        sleep(1)
        self.__rosbag.pause()
        return True

    def play_rosbag(self):
        self.__rosbag.play()

    def pause_rosbag(self):
        self.__rosbag.pause()

    def stop_rosbag(self):
        self.__rosbag.stop()

    def gateway_on(self):
        self.launch(domain="gateway", target="on")

    def gateway_off(self):
        self.launch(domain="gateway", target="off")
