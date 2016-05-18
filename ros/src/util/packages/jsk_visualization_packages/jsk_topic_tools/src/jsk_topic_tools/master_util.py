import rospy
import re
import os

previous_run_id = None

def isMasterAlive():
    """
    return True if master alive and return False if
    master is not alive
    """
    global previous_run_id
    try:
        # first check the host is available
        master = rospy.get_master()
        master_host = re.search('http://([a-zA-Z0-9\-_]*):', master.getUri()[2]).groups(1)[0]
        response = os.system("ping -W 10 -c 1 " + master_host + " > /dev/null")
        if response != 0:
            print "master machine looks down"
            return False
        master.getSystemState()
        run_id = rospy.get_param("/run_id")

        if not previous_run_id:
            previous_run_id = run_id
        if run_id != previous_run_id:
            print "run_id is not same"
            previous_run_id = run_id
            return False
        return True
    except Exception, e:
        return False
