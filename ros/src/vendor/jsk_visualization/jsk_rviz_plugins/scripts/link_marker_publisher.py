#!/usr/bin/env python

"""
Publish a visualization_marker for specified link
"""

import rospy
from visualization_msgs.msg import Marker
from xml.dom.minidom import parse, parseString

if __name__ == "__main__":
    rospy.init_node("link_marker_publisher")
    link_name = rospy.get_param("~link")
    rgb = rospy.get_param("~rgb", [1, 0, 0])
    alpha = rospy.get_param("~alpha", 1.0)
    scale = rospy.get_param("~scale", 1.02)
    robot_description = rospy.get_param("/robot_description")
    # Parse robot_description using minidom directly
    # because urdf_parser_py cannot read PR2 urdf
    doc = parseString(robot_description)
    links = doc.getElementsByTagName('link')
    mesh_file = None
    for link in links:
        if link_name == link.getAttribute('name'):
            visual_mesh = link.getElementsByTagName('visual').item(0).getElementsByTagName('mesh').item(0)
            mesh_file = visual_mesh.getAttribute('filename')
            break
    if not mesh_file:
        raise Exception("Cannot find link: {0}".format(link_name))
    pub = rospy.Publisher('~marker', Marker)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = link_name
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        marker.color.a = alpha
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.mesh_resource = mesh_file
        marker.frame_locked = True
        pub.publish(marker)
        rate.sleep()
