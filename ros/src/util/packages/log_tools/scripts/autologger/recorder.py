#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time, signal, argparse
import common
logger = common.genlogger()

def getargs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default=common.TMP_PATH)
    parser.add_argument("--prefix", default=common.BAG_PREFIX)
    parser.add_argument("--topics", type=str, nargs='+',
        default=["/velodyne_packets", "/image_raw", "/can_info"])
    parser.add_argument("--size", type=int, default=10)   # [GB]
    parser.add_argument("--all", action="store_true")
    args = parser.parse_args()
    args.dir = common.normpath(args.dir)
    common.gendir(args.dir) # create directory
    return args

if __name__ == '__main__':
    # signal handler for soft-kill
    is_shutdown = False
    def sigterm(): is_shutdown = True
    signal.signal(signal.SIGALRM, sigterm)
    logger.info("kill by Ctrl-C")
    # rosbag record command
    args = getargs()
    if args.all:
        topics = "--all"
        logger.info("record all topics")
    else:
        topics = " ".join(args.topics)
        logger.info("record topics: {}".format(topics))
    # prepare record
    prefix = "{}/{}".format(args.dir, args.prefix)
    logger.info("save directory: {}".format(args.dir))
    logger.info("rosbag prefix: {}_*.bag".format(prefix))
    # execute command
    cmd = "rosbag record --split --size {} -o {} {}"
    cmd = cmd.format(args.size*1024, prefix, topics)
    logger.info("execute command: {}".format(cmd))
    pid = common.execmd(cmd, blocking=False) # non-blocking
    while not is_shutdown: time.sleep(1)
    pid.kill()
