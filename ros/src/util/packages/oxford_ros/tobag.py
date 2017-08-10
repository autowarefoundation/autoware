#!/usr/bin/python

from __future__ import print_function
import sys
import rosbag


import sdk
from player import ImagePlayer, PosePlayer, Lidar2Player, Lidar3Player


def DatasetToRosbag (datasetDir, bagOutputPath, addLidar3=True, addLidar2=True):
    dataset = sdk.Dataset (datasetDir)
    bagOutput = rosbag.Bag(bagOutputPath, mode='w')
    
    imageSeqLen = len(dataset.getStereo())
    images = ImagePlayer.iteratorToMessage(dataset)
    i = 1
    for msg in images:
        if msg is None:
            continue
        bagOutput.write(ImagePlayer.topicName, msg, t=msg.header.stamp)
        print ("Images: {} / {}       ".format(i, imageSeqLen), end="\r")
        i += 1
    print ("\n")

    poses = PosePlayer.createPoseMessages(dataset)
    for ps in poses:
        curStamp = ps['geom'].header.stamp
        bagOutput.write(PosePlayer.topicName, ps['geom'], t=curStamp)
        bagOutput.write('/tf', ps['tf'], t=curStamp)
         
    if addLidar3:
        lidar3SeqLen = len(dataset.getMainLidar())
        lidar3s = Lidar3Player.iteratorToMessage(dataset)
        i = 1
        for msg in lidar3s:
            bagOutput.write(Lidar3Player.topicName, msg, t=msg.header.stamp)
            print ("Lidar 3: {} / {}       ".format(i, lidar3SeqLen), end="\r")
            i += 1
        print ("\n")

    if addLidar2:
        lidar2SeqLen = len(dataset.getLidar2D())
        lidar2s = Lidar2Player.iteratorToMessage(dataset)
        i = 1
        for msg in lidar2s:
            bagOutput.write(Lidar2Player.topicName, msg, t=msg.header.stamp)
            print ("Lidar 2: {} / {}       ".format(i, lidar2SeqLen), end="\r")
            i += 1
        print ("\n")
            
    bagOutput.close()
    


if __name__ == '__main__' :
    DatasetToRosbag(sys.argv[1], sys.argv[2])
