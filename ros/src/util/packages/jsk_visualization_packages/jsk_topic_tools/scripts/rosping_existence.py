#!/usr/bin/env python

"""
rosping_existence.py

check the existence of the ros nodes and publish the result to diagnostics
"""

import rospy
try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy

from rosnode import *
import sys
import roslaunch
import diagnostic_updater
import diagnostic_msgs

def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSNodeException("remote call failed: %s"%msg)
    return val

def ping(node_name, max_count=None, verbose=False):
    """
    Test connectivity to node by calling its XMLRPC API
    @param node_name: name of node to ping
    @type  node_name: str
    @param max_count: number of ping requests to make
    @type  max_count: int
    @param verbose: print ping information to screen
    @type  verbose: bool
    @return: True if node pinged
    @rtype: bool
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    node_api = get_api_uri(master,node_name)
    if not node_api:
        # print "cannot ping [%s]: unknown node" % node_name, file=sys.stderr
        return False

    timeout = 3.

    if verbose:
        print("pinging %s with a timeout of %ss"%(node_name, timeout))
    socket.setdefaulttimeout(timeout)
    node = ServerProxy(node_api)
    lastcall = 0.
    count = 0
    acc = 0.
    try:
        while True:
            try:
                count += 1
                start = time.time()
                pid = _succeed(node.getPid(ID))
                end = time.time()

                dur = (end-start)*1000.
                acc += dur
                

                if verbose:
                    print("xmlrpc reply from %s\ttime=%fms"%(node_api, dur))
                # 1s between pings
            except socket.error as e:
                # 3786: catch ValueError on unpack as socket.error is not always a tuple
                try:
                    # #3659
                    errnum, msg = e
                    if errnum == -2: #name/service unknown
                        p = urlparse.urlparse(node_api)
                        #print("ERROR: Unknown host [%s] for node [%s]"%(p.hostname, node_name), file=sys.stderr)
                    elif errnum == errno.ECONNREFUSED:
                        # check if node url has changed
                        new_node_api = get_api_uri(master,node_name, skip_cache=True)
                        if not new_node_api:
                            #print("cannot ping [%s]: unknown node"%node_name, file=sys.stderr)
                            return False
                        if new_node_api != node_api:
                            if verbose:
                                print("node url has changed from [%s] to [%s], retrying to ping"%(node_api, new_node_api))
                            node_api = new_node_api
                            node = ServerProxy(node_api)
                            continue
                         #print("ERROR: connection refused to [%s]"%(node_api), file=sys.stderr)
                    else:
                        pass
                        #print("connection to [%s] timed out"%node_name, file=sys.stderr)
                    return False
                except ValueError:
                    print("unknown network error contacting node: %s"%(str(e)))
            if max_count and count >= max_count:
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
            
    if verbose and count > 1:
        print("ping average: %fms"%(acc/count))
    return True

def checkNodeExistence(stat):
    global nodes
    result = {}
    have_dead = False
    for n in nodes:
        res = ping(n, max_count = 1, verbose = False)
        result[n] = res
        if not res:
            have_dead = True
    if have_dead:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                     "dead nodes: " + ", ".join([n for (n, res) 
                                                 in result.items() if not res]))
    else:
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                     "every node is alive")
    for n, res in result.items():
        stat.add(n, res)
    return stat
        
if __name__ == "__main__":
    rospy.init_node('rosping_existence')
    updater = diagnostic_updater.Updater()
    updater.setHardwareID(rospy.get_name())
    updater.add("node existence", checkNodeExistence)
    argv = rospy.myargv()
    # you can specify the list of the launch files
    launch_files = argv[1:]
    config = roslaunch.config.load_config_default(launch_files, 0)
    nodes = [n.namespace + n.name for n in config.nodes]
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        updater.update()
        r.sleep()
    
