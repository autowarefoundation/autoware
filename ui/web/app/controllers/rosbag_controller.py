#!/usr/bin/env python
# coding: utf-8

import rospy
import rosbag
from ros_command import ROSCommand


class ROSBAGController(object):
    def __init__(self):
        self.__bag = None

    def load(self, path):
        self.__bag = rosbag.Bag(path)
        return ROSCommand.popen(
            [
                "rosbag", "play", "--clock", path,
                "__name:=rosbag_play", "--pause"
            ])

    def play(self):
        return ROSCommand.call(
                [
                    "rosservice", "call",
                    "/rosbag_play/pause_playback", "false"
                ])

    def pause(self):
        return ROSCommand.call(
            [
                "rosservice", "call",
                "/rosbag_play/pause_playback", "true"
            ])

    def stop(self):
        self.__bag = None
        return ROSCommand.call(
            ["rosnode", "kill", "/rosbag_play"])

    def get_progress(self):
        if self.__bag is not None:
            current_time = self.__get_current_time()
            if current_time is not None:
                return current_time - self.__get_start_time(), self.__get_end_time() - self.__get_start_time()
            else:
                self.__bag = None
                return None, None
        else:
            return None, None

    def __get_current_time(self):
        if "/clock" in list(map(lambda x: x[0], rospy.get_published_topics())):
            try:
                secs, nsecs = self.__get_subscribe_once("/clock")
            except:
                return None
            return float(".".join(list(map(str, [secs, nsecs]))))
        else:
            return None

    def __get_start_time(self):
        if self.__bag is not None:
            return self.__bag.get_start_time()
        else:
            return None

    def __get_end_time(self):
        if self.__bag is not None:
            return self.__bag.get_end_time()
        else:
            return None
