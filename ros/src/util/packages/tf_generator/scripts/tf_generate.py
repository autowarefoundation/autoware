#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
import geometry_msgs.msg


def callback(data):
    points = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
    counter = len(list(points))

    coordinate = [0, 0, 0]
    for index in range(len(coordinate)):
        points = pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True)
        coordinate[index] = -1*(sum(map(lambda x: x[index], points)) / counter)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "map"

    static_transformStamped.transform.translation.x = float(coordinate[0])
    static_transformStamped.transform.translation.y = float(coordinate[1])
    static_transformStamped.transform.translation.z = float(coordinate[2])

    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

    sys.stderr.write("world_to_map tf is running. params: %f %f %f\n" %(coordinate[0], coordinate[1], coordinate[2]))

def listener():
    rospy.init_node('listener', anonymous =True)
    rospy.Subscriber("points_map", PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
