from __future__ import division
import rospy
import tf
import sys


source_frame = 'world'
target_frame = 'camera1'
fps = 20.0


if __name__ == '__main__' :
    
    rospy.init_node ('tf_recorder')
    listener = tf.TransformListener ()
    
    rate = rospy.Rate (fps)
    timeout = rospy.Duration (1/fps)
    
    while not rospy.is_shutdown ():
        try:
            t = listener.getLatestCommonTime(source_frame, target_frame)
#             listener.waitForTransform(target_frame, source_frame, t, timeout)
            position, rotation = listener.lookupTransform(source_frame, target_frame, t)
            print ( "{} {} {} {} {} {} {} {}".format(
                # Time
                t.to_sec(),
                # Position in X Y Z 
                position[0], position[1], position[2],
                # Quaternion in X Y Z W
                rotation[0], rotation[1], rotation[2], rotation[3] 
                ) )
        except tf.Exception:
            print("None")
        
        sys.stdout.flush()
        rate.sleep()