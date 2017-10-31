from __future__ import division
from orbndt import Pose, PoseTable
import numpy as np


def findMatches (grndTbl1, grndTbl2):
    # For first map, start from behind
    # For later map, start from front
    
    
    pass


if __name__ == '__main__' :
    
    # Map loading
    _map1m = np.loadtxt('/home/sujiwo/Data/Road_Datasets/2016-01-21-15-12-49/s1-map.csv')
    _map2m = np.loadtxt('/home/sujiwo/Data/Road_Datasets/2016-01-21-15-12-49/s2-map.csv')
    
    # For ground truth
    tmp1g = np.zeros((_map1m.shape[0], 8))
    tmp2g = np.zeros((_map2m.shape[0], 8))
    tmp1g[:,0] = _map1m[:,0]
    tmp1g[:,1:] = _map1m[:,8:]
    tmp2g[:,0] = _map2m[:,0]
    tmp2g[:,1:] = _map2m[:,8:]
    mapGroundTruth1 = PoseTable.loadFromArray(tmp1g)
    mapGroundTruth2 = PoseTable.loadFromArray(tmp2g)

    # For ORB Poses
    tmp1o = np.zeros(tmp1g.shape)
    tmp2o = np.zeros(tmp1g.shape)
    tmp1o[:,0] = _map1m[:,0]
    tmp1o[:,1:] = _map1m[:,1:8]
    tmp2g[:,0] = _map2m[:,0]
    tmp2g[:,1:] = _map2m[:,1:8]
    mapOrb1 = PoseTable.loadFromArray(tmp1o)
    mapOrb2 = PoseTable.loadFromArray(tmp2o)

    
    print ('XXX')