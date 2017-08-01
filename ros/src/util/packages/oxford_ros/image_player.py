
import sys
import rospy
import cv2
import cv_bridge
import time
import numpy as np

from sensor_msgs.msg import Image as ImageMsg
from rosgraph_msgs.msg import Clock

import sdk


if __name__ == '__main__' :
    
    rospy.init_node ('oxford_image_player', anonymous=True)
    cvbridge = cv_bridge.CvBridge()
    publisher = rospy.Publisher ('stereo_center', ImageMsg, queue_size=1)
    
    # XXX: Need to get dataset directory from elsewhere
    testdata = sdk.Dataset(sys.argv[1])
    cam_ts = testdata.getStereo ()
    cameraModel = sdk.CameraModel ('models', sdk.CameraModel.cam_stereo_center)
    
    for captures in cam_ts :
        image_ctr = cv2.imread(captures['center'], cv2.IMREAD_ANYCOLOR)
        image_ctr = cv2.cvtColor(image_ctr, cv2.COLOR_BAYER_GR2BGR)
        image_ctr = cameraModel.undistort (image_ctr)
        msg = cvbridge.cv2_to_imgmsg(image_ctr, 'bgr8')
        publisher.publish(msg)
        time.sleep(0.1)
        if (rospy.is_shutdown()):
            break
