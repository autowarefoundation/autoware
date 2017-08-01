
import os
import re
import numpy as np
from datetime import datetime as dt


class Dataset:
    
#     poseModeGps = 1
#     poseModeIns = 2
#     poseModeVO  = 3

    OriginCorrectionEasting = -620248.53
    OriginCorrectionNorthing = -5734882.47
    
    def __init__ (self, datadir):
        if (not os.path.isdir(datadir)):
            raise IOError 
        self.path = datadir

    def getTimestamp (self, name, raw=False):
        abspath = self.path + '/' + name + '.timestamps'
        fd = open(abspath)
        tslist = []
        for l in fd:
            tokens = l.split()
            if raw==True:
                tslist.append(tokens[0])
            else:
                datetime = dt.utcfromtimestamp(int(tokens[0])/1000000)
                tslist.append(datetime)
        return tslist
    
    # Returns file names
    def getStereo (self):
        timestamps = self.getTimestamp('stereo', raw=True)
        fileList = []
        ctrname = self.path + '/stereo/centre'
        lftname = self.path + '/stereo/left'
        rhtname = self.path + '/stereo/right'
        for ts in timestamps :
            rcatch = {
                    'timestamp' : float(ts) / 1000000.0,    # in seconds
                    'center' : ctrname + '/' + ts + '.png',
                    'left'   : lftname + '/' + ts + '.png',
                    'right'  : rhtname + '/' + ts + '.png'
                }
            fileList.append(rcatch)
        return fileList
    
    def getMainLidar (self):
        timestamps = self.getTimestamp('ldmrs', raw=True)
        filelist = []
        for ts in timestamps:
            f = {'timestamp': float(ts)/1000000.0, 'path': self.path + '/ldmrs/' + ts + '.bin'}
            filelist.append (f)
        return filelist
    
    def getLidar2D (self, name='front'):
        timestamps = self.getTimestamp("lms_{}".format(name), raw=True)
        filelist = []
        for ts in timestamps:
            f = {'timestamp': float(ts)/1000000.0, 'path': self.path + '/lms_front/' + ts + '.bin'}
            filelist.append (f)
        return filelist
            
            
    
    def getGps (self, useOriginCorrection=True):
        gpsTbl = np.loadtxt(self.path+'/gps/gps.csv',
            skiprows=1,
            delimiter=',',
            usecols=[0,9,8,4])
        gpsTbl[:,0] /= 1000000.0
        if useOriginCorrection:
            gpsTbl[:,1] += Dataset.OriginCorrectionEasting
            gpsTbl[:,2] += Dataset.OriginCorrectionNorthing
        return gpsTbl
    
    def getIns (self, useOriginCorrection=True):
        # Get timestamp, easting, northing, altitude, roll, pitch, yaw
        insTbl = np.loadtxt(self.path+'/gps/ins.csv', 
            skiprows=1, 
            delimiter=',', 
            usecols=[0,6,5,4,12,13,14])
        insTbl[:,0] /= 1000000.0
        if useOriginCorrection:
            insTbl[:,1] += Dataset.OriginCorrectionEasting
            insTbl[:,2] += Dataset.OriginCorrectionNorthing
        # Correct pitch & yaw rotations to more sensible ROS convention; otherwise you will have problems later
        insTbl[:,5] = -insTbl[:,5]
        insTbl[:,6] = -insTbl[:,6]
        return insTbl
    
#     def getExtrinsics (self, name):
#         path = self.path + '/ex' + name + '.txt'
#         return np.fromfile(path)