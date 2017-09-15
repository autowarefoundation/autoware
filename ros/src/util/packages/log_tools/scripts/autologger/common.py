#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, datetime, hashlib
import subprocess, shlex, logging

HOME_PATH = os.path.expanduser("~/autologger")
LOG_PATH = "{}/log".format(HOME_PATH)
TMP_PATH = "{}/tmp".format(HOME_PATH)
BAG_PATH = "{}/bag".format(HOME_PATH)
BAG_PREFIX = "autologger"

def genhash(path):
    h = hashlib.md5(open(path, "rb").read())
    return h.hexdigest()

def chkhash(hsh, path):
    return (hsh == genhash(path))

def normpath(path):
    return os.path.normpath(os.path.expanduser(path))

def execmd(cmd, blocking=True):
    p = subprocess.Popen(shlex.split(cmd),
        stderr=subprocess.PIPE, stdout=subprocess.PIPE)
    if blocking: p.wait()
    return p

def gendir(path):
    if not os.path.exists(path): os.makedirs(path)

def getutime():
    return datetime.datetime.now().strftime("%s.%f")

def getdate(ns=False):
    fmt = "%Y-%m-%d-%H-%M-%S"
    fmt = fmt + "-%f" if ns else fmt    # add nsec
    return datetime.datetime.now().strftime(fmt)

def genlogger():
    # logger settings
    name = os.path.basename(sys.argv[0])
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    # formatter
    fmt = "%(asctime)s : %(lineno)d : %(levelname)s : %(message)s"
    fmtr = logging.Formatter(fmt)
    # file handler
    gendir(LOG_PATH)
    fn = "{}/{}_{}.log".format(LOG_PATH, name, getdate())
    fh = logging.FileHandler(fn)
    fh.setFormatter(fmtr)
    logger.addHandler(fh)
    # stream handler
    sh = logging.StreamHandler()
    logger.addHandler(sh)
    sh.setFormatter(fmtr)
    return logger
