#!/usr/bin/python

import sys
import rosbag

import sdk
from player import ImagePlayer, PosePlayer, Lidar2Player, Lidar3Player


def DatasetToRosbag (datasetDir, bagOutputPath):
    dataset = sdk.Dataset (datasetDir)
    bagOutput = rosbag.Bag(bagOutputPath, mode='w')
    
    images = ImagePlayer.iteratorToMessage(dataset)
    for msg in images:
        bagOutput.write(ImagePlayer.topicName, msg, t=msg.header.stamp)
               
    poses = PosePlayer.createPoseMessages(dataset)
    for ps in poses:
        curStamp = ps['geom'].header.stamp
        bagOutput.write(PosePlayer.topicName, ps['geom'], t=curStamp)
        bagOutput.write('/tf', ps['tf'], t=curStamp)
         
    lidar3s = Lidar3Player.iteratorToMessage(dataset)
    for msg in lidar3s:
        bagOutput.write(Lidar3Player.topicName, msg, t=msg.header.stamp) 
        
    lidar2s = Lidar2Player.iteratorToMessage(dataset)
    for msg in lidar2s:
        bagOutput.write(Lidar2Player.topicName, msg, t=msg.header.stamp)
            
    bagOutput.close()
    


if __name__ == '__main__' :
    DatasetToRosbag(sys.argv[1], sys.argv[2])
