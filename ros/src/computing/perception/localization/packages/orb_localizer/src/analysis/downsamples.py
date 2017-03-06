#!/usr/bin/python2.7

from __future__ import division
import os
import sys
from multiprocessing import dummy as mp
from multiprocessing import Lock
from mutex import mutex
from copy import copy

import cv2

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError


logger = None


class DataWriter:
    def __init__ (self, targetDir, fps=10.0):
        
        # Create target directory and pose log
        if not os.path.exists(targetDir) :
            os.mkdir (targetDir)
        self.poseFd = open(targetDir+"/pose.csv", 'w')
        self.path = targetDir
        
        self.currentImage = None
        self.currentPose = None
        self.fps = fps
        self.bridge = CvBridge()
        self.currentTimestamp = 0.0

        # Prepare multiprocessing
        self.dataEx = Lock()
        self.process = mp.Process(target=self.start)
        self.end = False
        self.process.start()
    
    def start (self):
        rate = rospy.Rate(self.fps)
        counter = 0
        
        while (self.end == False):
            if self.currentImage is not None and self.currentPose is not None :
                
                self.dataEx.acquire()
                cpImg = copy (self.currentImage)
                cpPose = copy (self.currentPose)
                self.dataEx.release()
                
                curImageName = "{0:06d}.jpg".format (counter)
                
                cv2.imwrite (self.path + "/"+curImageName, self.currentImage)
                poseStr = "{:.6f} {} {} {} {} {} {} {}".format(
                    rospy.Time.now().to_sec(),
                    cpPose.position.x,
                    cpPose.position.y,
                    cpPose.position.z,
                    cpPose.orientation.x,
                    cpPose.orientation.y,
                    cpPose.orientation.z,
                    cpPose.orientation.w)
                self.poseFd.write(poseStr+"\n") 
                
                counter += 1
            rate.sleep()
    
    def close (self):
        self.end = True
        self.poseFd.close()


def ImageCallback (imageMsg):
    if logger is None:
        return
    # Convert to OpenCV format
    logger.dataEx.acquire()
    cImage = logger.bridge.imgmsg_to_cv2(imageMsg, 'mono8')
    logger.currentImage = copy (cImage)
    logger.dataEx.release()
    

def PoseCallback (poseMsg):
    if logger is None:
        return
    
    # Put into datawriter
    logger.dataEx.acquire()
    logger.currentTimestamp = poseMsg.header.stamp.to_sec()
    logger.currentPose = copy(poseMsg.pose)
    logger.dataEx.release()
    

if __name__ == '__main__' :
    rospy.init_node ("ImagePoseSubscriber", anonymous=True)
    
    logger = DataWriter(sys.argv[1]) 
    
    rospy.Subscriber ("/camera/image_hs", Image, ImageCallback, queue_size=20)
    rospy.Subscriber ("/filtered_ndt_current_pose", PoseStamped, PoseCallback, queue_size=100)
    
    rospy.spin()
    logger.close()
    
    pass
