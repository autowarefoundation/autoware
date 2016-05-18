#!/usr/bin/env python


import sys
import rospy
import subprocess
import unittest
import rostest

class CloseWaitCheck(unittest.TestCase):
    def __init__(self, *args):
        super(CloseWaitCheck, self).__init__(*args)
        rospy.init_node("ClosWaitCheck")
    def setUp(self):
        True

    # this works for hydro, but not for groovy https://github.com/ros/ros_comm/issues/325
    @unittest.expectedFailure
    def test_close_wait_check(self):
        rospy.loginfo("check close_wait...")
        num = 0
        for i in range(5):
            num = int(subprocess.check_output("lsof -nl | grep rosmaster | grep CLOSE_WAIT | wc -l", shell=True))
            rospy.loginfo("number of close_wait is ... %d (%d)"%(num, i))
            self.assert_(num>1, "some socket is not closed, found CLOSE_WAIT")
            subprocess.call("rosnode kill sample_topic_buffer_server", shell=True)
            rospy.sleep(rospy.Duration(1.0))
        return True



if __name__ == '__main__':
    try:
        rostest.run('rostest', "CloseWaitCheck", CloseWaitCheck, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

# while [ t ]; do
#     NUM_OF_PROCESS=`lsof -nl | grep rosmaster | grep CLOSE_WAIT | wc -l`
#     echo $NUM_OF_PROCESS
#     if [ $NUM_OF_PROCESS != 0 ]; then
#         exit -1
#     fi
#     sleep 5
#     rosnode kill sample_topic_buffer_server
# done


