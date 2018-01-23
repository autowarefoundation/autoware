#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import tf
import yaml
import paho.mqtt.client as mqtt
from geometry_msgs.msg import PoseStamped

mqtt_publisher = None;
mqtt_config = None;

def publish_current_pose(msg):
    current_pose_array = msg.payload.split(",")
    current_pose = PoseStamped()

    current_pose.header.frame_id = "/map"
    current_pose.header.stamp = rospy.Time.now()
    current_pose.pose.position.x = float(current_pose_array[0])
    current_pose.pose.position.y = float(current_pose_array[1])
    current_pose.pose.position.z = float(current_pose_array[2])
    current_pose.pose.orientation.x = float(current_pose_array[3])
    current_pose.pose.orientation.y = float(current_pose_array[4])
    current_pose.pose.orientation.z = float(current_pose_array[5])
    current_pose.pose.orientation.w = float(current_pose_array[6])
    
    mqtt_publisher.publish(current_pose)
    return current_pose

def publish_tf(current_pose):
    br = tf.TransformBroadcaster()
    br.sendTransform((current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z),
                     (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/base_link",
                     "/map")

def on_connect(client, userdata, flags, respons_code):
    rospy.loginfo("ON CONNECT TO MQTT BROKER.")
    current_pose_topic = "vehicle/" + str(mqtt_config['mqtt']['VEHICLEID']) + "/current_pose"
    client.subscribe(current_pose_topic)

def on_message(client, userdata, msg):
    current_pose = publish_current_pose(msg)
    publish_tf(current_pose)

if __name__ == '__main__':
    rospy.init_node("mqtt_pose_receiver")
    mqtt_publisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

    config_file_path = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../mqtt_config.yml'))
    f = open(config_file_path, "r+")
    mqtt_config = yaml.load(f)
    rospy.loginfo("[MQTT BROKER] ADDRESS: " + mqtt_config['mqtt']['ADDRESS'] + ", PORT: " + str(mqtt_config['mqtt']['PORT']))

    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqtt_config['mqtt']['ADDRESS'], port=mqtt_config['mqtt']['PORT'], keepalive=mqtt_config['mqtt']['TIMEOUT'])
    client.loop_start()

    rospy.spin()
