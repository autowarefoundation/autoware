import os
import yaml



def userhome(path = ""):
    return os.path.abspath(os.path.join(os.path.expanduser("~"), path))

def autoware(path = ""):
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 8 * "../", path))

def package(path = ""):
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 3 * "../", path))

def plugins():
    return os.path.join(package(), "plugins")

def profile(profile = ""):
    return os.path.join(package(), "profiles", profile)

def parentpath(path):
    return os.path.dirname(path)

def makedirs(path, mode=0777, exist_ok=False): # workaround in python2
    if not (exist_ok and os.path.exists(path)): os.makedirs(path, mode)

def listfiles(rootpath, relative=False):
    filelist = []
    for currpath, dirnames, filenames in os.walk(rootpath):
        if relative:
            currpath = os.path.relpath(currpath, rootpath)
            if currpath == ".":
                currpath = ""
        for filename in filenames:
            filelist.append(os.path.join(currpath, filename))
    return filelist

def envpath(path):
    patterns = \
    [
        (os.environ['HOME'], "$(env HOME)")
    ]
    for pattern, replace in patterns:
        if path.startswith(pattern):
            return replace + path[len(pattern):]
    return path
