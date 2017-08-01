#!/usr/bin/python

import numpy as np
import multiprocessing
import rospy
import rospkg
from tf import TransformBroadcaster
from tf import transformations


class OxTfPublisher:
    def __init__ (self, extrName, from_frame, to_frame):
        self.publisher = TransformBroadcaster()
        pkgpack = rospkg.RosPack()
        mypath = pkgpack.get_path('oxford_ros')
        extrinsicPath = mypath + '/extrinsics/' + extrName + '.txt'
        vector = np.fromfile(extrinsicPath, count=6, sep=' ')
        self.from_name = from_frame
        self.to_name = to_frame
        self.position = [vector[0], vector[1], vector[2]]
        self.orientation = transformations.quaternion_from_euler (
            vector[3], vector[4], vector[5])
        
    def publish (self):
        self.publisher.sendTransform(
            self.position, 
            self.orientation, 
            rospy.Time.now(), 
            self.from_name, 
            self.to_name)



if __name__ == '__main__' :
    
    rospy.init_node ('oxford_vehicle_tf')
    cam2ins = OxTfPublisher ('ins', 'camera', 'ins')
    cam2lidar = OxTfPublisher ('ldmrs', 'camerax', 'ldmrs')
    
    rate = rospy.Rate(20)
    while (not rospy.is_shutdown()):
        cam2lidar.publish()
        cam2ins.publish()
        rate.sleep()