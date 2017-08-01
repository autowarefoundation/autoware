#!/usr/bin/python


import sys
import sdk
from tf import transformations as trafo
import numpy as np
import bisect


base_link2lms_front_tr = [0.126, 3.746, -0.051]
base_link2lms_front_qr = [0.524, 0.512, 0.483, -0.480]
# limit mapping time length, in seconds
timeLimit = 1000


def interpolatePose (targetTimestamp, poseTimestamps, poses):
    u_1 = bisect.bisect_left(poseTimestamps, targetTimestamp)-1
    # Extreme case
    if u_1 >= (len(poseTimestamps)-1):
        return poses[u_1]
        
    u_2 = u_1 + 1
    
    fraction = (targetTimestamp - poseTimestamps[u_1]) / \
        (poseTimestamps[u_2] - poseTimestamps[u_1])

    p1 = poses[u_1]
    p2 = poses[u_2]
    movement = p2[0:3] - p1[0:3]
    translation = p1[0:3] + fraction*movement
    rotation = trafo.quaternion_slerp(p1[3:], p2[3:], fraction)
    return np.array([translation[0], translation[1], translation[2],
        rotation[0], rotation[1], rotation[2], rotation[3]])
    

def readScan (path):
    scanMat = np.fromfile(path, np.double)
    scanMat = scanMat.reshape ((len(scanMat) // 3,3)).astype(np.float32)
    scanz = np.zeros((scanMat.shape[0], 4), dtype=np.float32)
    scanz[:,0:2] = scanMat[:,0:2]
    # X Axis from scans is negated to comform with right-hand convention
    scanz[:,0] = -scanz[:,0]
    scanz[:,3] = 1
    return scanz


def buildTransformationMatrix (translation, rotationQuat):
#     trm = trafo.translation_matrix (translation)
    rtm = trafo.quaternion_matrix (rotationQuat)
    tfmat = np.eye(4, dtype=np.float32)
    tfmat[:3,:3] = rtm[:3,:3].transpose()
    tfmat[:3,3] = (-rtm[:3,:3].transpose()).dot(translation)
    return tfmat
    


def buildMap(datasetDir):
    global timeLimit
    
    dataset = sdk.Dataset (datasetDir)
    scanList = []
    
    posesTbl = dataset.getIns ()
    poses = np.zeros((len(posesTbl), 7))
    for i in range(len(poses)):
        r = posesTbl[i]
        poses[i, 0:3] = r[1:4]
        poses[i, 3:] = trafo.quaternion_from_euler(r[4], r[5], r[6]) 
    
    lidarFileList = dataset.getLidar2D ()
    poseTimestamps = list(posesTbl[:,0])
    
    laserCorrection = buildTransformationMatrix(base_link2lms_front_tr, base_link2lms_front_qr)
    laserCorrectionInv = np.linalg.inv(laserCorrection)
    
    for fip in range(len(lidarFileList)) :
        
        if timeLimit > 0:
            if (lidarFileList[fip]['timestamp'] > lidarFileList[0]['timestamp']+timeLimit):
                break
        
        scan = readScan(lidarFileList[fip]['path'])
        # This pose represent vehicle pose in world coordinate.
        # We want transformation from laser scanner to world         
        pose_base = interpolatePose(lidarFileList[fip]['timestamp'], poseTimestamps, poses)
        pose_base_mat = buildTransformationMatrix([pose_base[0], pose_base[1], pose_base[2]], 
            [pose_base[3], pose_base[4], pose_base[5], pose_base[6]])
        trans = np.linalg.inv(pose_base_mat).dot(laserCorrectionInv)
        scanList.append({'scan':scan, 'trans':trans})
        
    numOfPoints = sum([sc['scan'].shape[0] for sc in scanList])
    pcMap = np.zeros((numOfPoints, 3), dtype=np.float32)
    print ("Found {} points".format(numOfPoints))
    
    i = 0
    for scan in scanList:
        ptList = scan['scan']
        for p in range(ptList.shape[0]):
            pointInScan = ptList[p]
            pointInWorld = scan['trans'].dot(pointInScan)
            pcMap[i] = pointInWorld[0:3]
            i+=1
    
    return pcMap



if __name__ == '__main__' :
    datadir=sys.argv[1]
    pcmap = buildMap (datadir)
    
    pass