#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, shutil, time, glob
import argparse, multiprocessing
import requests
import common
logger = common.genlogger()

def getargs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="localhost")
    parser.add_argument("--port", default="3000")
    parser.add_argument("--dir", default=common.TMP_PATH)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--retry", default=5)
    parser.add_argument("--wait", default=10) # [sec]
    parser.add_argument("--rename", action="store_true")
    args = parser.parse_args()
    args.dir = common.normpath(args.dir)
    return args

def status(url):
    return requests.get(url).json()

def upload(url, path):
    # post form
    utime = common.getutime()
    hsh = common.genhash(path)
    form = { "utime": utime, "hash": hsh }
    # post files (single data)
    files = { "file": open(path, 'rb') }
    # upload
    logger.info("upload start: {}".format(path))
    logger.debug("unixtime: {}, hash: {}".format(utime, hsh))
    return requests.post(url, data=form, files=files).json()

def worker(url, path, retry=-1, wait=1, rename=False, rnfmt="{}.uploaded"):
    for i in xrange(retry+1):
        try:
            res = upload(url, path)
            if not res["success"]: raise Exception
            logger.info("upload complete: {}".format(path))
            if rename:  # rename mode
                dstpath = rnfmt.format(path)
                shutil.move(path, dstpath)
                logger.info("rename file -> {}".format(dstpath))
            else:   # remove mode
                os.remove(path)
                logger.info("remove file: {}".format(path))
            return
        except Exception as e:
            logger.info("upload error: {}".format(e))
            logger.info("retry upload ({}): {}".format(i, path))
            time.sleep(wait)
    logger.info("failed upload!: {}".format(path))

if __name__ == "__main__":
    args = getargs()
    url = "http://{}:{}".format(args.ip, args.port)
    urlup = "{}/upload".format(url)
    urlst = "{}/status".format(url)
    logger.info("URL = {}".format(url))
    logger.info("watching directory = {}".format(args.dir))
    # check connection
    while True:
        try:
            # use to check connection only
            res = status(urlst)
            logger.info("connection success, status = {}".format(res))
            break
        except Exception as e:
            logger.error("connection error = {}".format(e))
            time.sleep(args.wait)
    # watch and upload
    queue = {}  # { path : { job, started } }
    while True:
        time.sleep(args.wait)   # loop rate
        # watch direcory
        paths = glob.glob("{}/{}".format(args.dir, "*.bag"))
        if len(paths) == 0:
            logger.info("no file ...")
            continue
        # prepare jobs
        for path in paths:
            path = common.normpath(path)
            if not path in queue.keys():
                process = multiprocessing.Process(target=worker,
                    args=(urlup, path, args.retry, args.wait, args.rename,))
                queue[path] = { "process": process, "started": False }
        # start jobs
        exeq = queue.items()[:args.threads]
        logger.info("queue size = {}, threads = {}".format(len(queue), len(exeq)))
        for path in [ path for path, job in exeq if not job["started"] ]:
            queue[path]["process"].start()
            queue[path]["started"] = True
        # delete finished jobs
        [ queue.pop(path) for path, job, in queue.items()
            if not job["process"].is_alive() and job["started"] ]
