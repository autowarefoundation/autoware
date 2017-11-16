#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flask import Flask, request, json
import os, argparse
import common

args = None
app = Flask(__name__)
logger = common.genlogger()

def getargs():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="0.0.0.0") # default: listen on all ip
    parser.add_argument("--port", default="3000")
    parser.add_argument("--limit", type=int, default=12) # [GB]
    parser.add_argument("--dir", default=common.BAG_PATH)
    args = parser.parse_args()
    args.dir = common.normpath(args.dir)
    common.gendir(args.dir)  # create directory
    return args

@app.route("/status", methods=["GET"])
def status():
    res = { "limit" : args.limit, "dir" : args.dir }
    return json.dumps(res)

@app.route("/upload", methods=["POST"])
def upload():
    reqform = request.form
    reqfile = request.files["file"]
    utime, hsh = reqform["utime"], reqform["hash"]
    # save file
    savepath = "{}/{}".format(args.dir, reqfile.filename)
    logger.info("receive file: {}".format(reqfile.filename))
    logger.debug("unixtime: {}, hash: {}".format(utime, hsh))
    logger.info("-> save path: {}".format(savepath))
    reqfile.save(savepath)
    # check hash
    res = { "success" : common.chkhash(reqform["hash"], savepath) }
    if not res["success"]:
        logger.error("hash check failed, remove file: {}".format(savepath))
        os.remove(savepath)
    return json.dumps(res)

if __name__ == "__main__":
    args = getargs()
    app.config['MAX_CONTENT_LENGTH'] = args.limit * 1024**3
    app.run(host=args.ip, port=args.port)
