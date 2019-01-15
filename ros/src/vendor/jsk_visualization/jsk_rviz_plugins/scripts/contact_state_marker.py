#!/usr/bin/env python

"""
Colorize link according to hrpsys_ros_bridge/ContactState using
visualization_msgs/Marker
"""

import rospy
from hrpsys_ros_bridge.msg import ContactState, ContactStateStamped, ContactStatesStamped
from visualization_msgs.msg import Marker, MarkerArray
from xml.dom.minidom import parse, parseString
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from jsk_rviz_plugins.cfg import ContactStateMarkerConfig
from dynamic_reconfigure.server import Server
from urdf_parser_py.urdf import URDF

g_config = None
def config_callback(config, level):
    global g_config
    g_config = config
    return config

def find_mesh(link_name):
    "find mesh file from specified link_name"
    global g_config
    for link in g_links:
        if link_name == link.getAttribute('name'):
            visual = link.getElementsByTagName('visual').item(0)
            visual_mesh = visual.getElementsByTagName('mesh').item(0)
            if len(visual.getElementsByTagName("origin")) > 0:
                origin = visual.getElementsByTagName("origin").item(0)
                pose = Pose()
                if origin.getAttribute("xyz"):
                    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in origin.getAttribute("xyz").split()]
                if origin.getAttribute("rpy"):
                    r, p, y = [float(v) for v in origin.getAttribute("rpy").split()]
                    q = quaternion_from_euler(r, p, y)
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
                    return (visual_mesh.getAttribute('filename'), pose)
            else:
                pose = Pose()
                pose.orientation.w = 1
                return (visual_mesh.getAttribute('filename'), pose)
    raise Exception("Cannot find link: {0}".format(link_name))

def callback(msgs):
    "msgs = ContactStatesStamped"
    global g_config
    if g_config.use_parent_link:
        urdf_robot = URDF.from_parameter_server()
    marker_array = MarkerArray()
    for msg, i in zip(msgs.states, range(len(msgs.states))):
        marker = Marker()
        link_name = msg.header.frame_id
        if g_config.use_parent_link:
            # lookup parent link
            chain = urdf_robot.get_chain(urdf_robot.get_root(), link_name)
            link_name = chain[-3]
        mesh_file, offset = find_mesh(link_name)
        marker.header.frame_id = link_name
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        if msg.state.state == ContactState.ON:
            marker.color.a = g_config.on_alpha
            marker.color.r = g_config.on_red
            marker.color.g = g_config.on_green
            marker.color.b = g_config.on_blue
        elif msg.state.state == ContactState.OFF:
            marker.color.a = g_config.off_alpha
            marker.color.r = g_config.off_red
            marker.color.g = g_config.off_green
            marker.color.b = g_config.off_blue
        marker.scale.x = g_config.marker_scale
        marker.scale.y = g_config.marker_scale
        marker.scale.z = g_config.marker_scale
        marker.pose = offset
        marker.mesh_resource = mesh_file
        marker.frame_locked = True
        marker.id = i
        if msg.state.state == ContactState.OFF:
            if not g_config.visualize_off:
                marker.action = Marker.DELETE
        marker_array.markers.append(marker)
    pub.publish(marker_array)

if __name__ == "__main__":
    rospy.init_node("contact_state_marker")
    rgb = rospy.get_param("~rgb", [1, 0, 0])
    alpha = rospy.get_param("~alpha", 0.5)
    scale = rospy.get_param("~scale", 1.02)
    robot_description = rospy.get_param("/robot_description")
    
    # Parse robot_description using minidom directly
    # because urdf_parser_py cannot read PR2 urdf
    doc = parseString(robot_description)
    g_links = doc.getElementsByTagName('link')
    srv = Server(ContactStateMarkerConfig, config_callback)
    pub = rospy.Publisher('~marker', MarkerArray)
    sub = rospy.Subscriber("~input", ContactStatesStamped, callback)
    rospy.spin()
