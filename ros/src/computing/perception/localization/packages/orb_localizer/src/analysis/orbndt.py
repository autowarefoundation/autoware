from __future__ import division
import numpy as np
import datetime
import rosbag
import rospy
from copy import copy, deepcopy
from exceptions import KeyError, ValueError
from segway_rmp.msg import SegwayStatusStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, tan, exp
from tf import transformations as trafo


ndtFName = '/home/sujiwo/ORB_SLAM/Data/20151106-1/ndt.csv'
orbFName = '/home/sujiwo/ORB_SLAM/Data/20151106-1/orb-slam.csv'



def ClosestPointInLine (PointA, PointB, pointChk, ifPointInLine=None):
    ap = pointChk.coord() - PointA.coord()
    ab = PointB.coord() - PointA.coord()
    cs = (np.dot(ap, ab) / np.dot(ab, ab))
    ptx = PointA.coord() + ab * cs
    retval = Pose(pointChk.timestamp, \
        ptx[0], ptx[1], ptx[2], \
        pointChk.qx, pointChk.qy, pointChk.qz, pointChk.qw)
    if ifPointInLine is None :
        return retval
    else :
        return retval, cs

class Pose :
    def __init__ (self, t=0, _x=0, _y=0, _z=0, _qx=0, _qy=0, _qz=0, _qw=1):
        try:
            self.timestamp = t[0]
            self.x = t[1]
            self.y = t[2]
            self.z = t[3]
            self.qx = t[4]
            self.qy = t[5]
            self.qz = t[6]
            self.qw = t[7]
        except (TypeError, IndexError):
            self.timestamp = t
            self.x = _x
            self.y = _y
            self.z = _z
            self.qx = _qx
            self.qy = _qy
            self.qz = _qz
            self.qw = _qw
            
    def plot (self, size=50, **kwargs):
        return plt.scatter(self.x, self.y, s=size, linewidths=0, **kwargs)
            
    @staticmethod
    # RPY must be in Radian
    def xyzEuler (x, y, z, roll, pitch, yaw, timestamp=0):
        pose = Pose (timestamp, x, y, z)
        qt = trafo.quaternion_from_euler(roll, pitch, yaw)
        pose.qx = qt[0]
        pose.qy = qt[1]
        pose.qz = qt[2]
        pose.qw = qt[3]
        return pose
    
    def quaternion (self):
        return np.array([self.qx, self.qy, self.qz, self.qw])
            
    def __str__ (self):
        return "X={}, Y={}, Z={}, Qx={}, Qy={}, Qz={}, Qw={}".format(self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
    
    def time (self):
        return datetime.datetime.fromtimestamp(self.timestamp)
    
    def offsetTime (self, t):
        self.timestamp += t
        
    def coord (self):
        return np.array([self.x, self.y, self.z])
            
    def rot (self):
        return np.array([self.qx, self.qy, self.qz, self.qw])
        
    def __sub__ (self, p1):
        return np.array([self.x-p1.x, self.y-p1.y, self.z-p1.z])

    # Only calculate movement, but does not overwrite the values
    # currentTimestamp must be in second
    def segwayMove (self, currentTimestamp, leftWheelVelocity, rightWheelVelocity, yawRate):
        # this is minimum speed to consider yaw changes (ie. yaw damping)
        minSpeed = 0.025        
        
        v = (leftWheelVelocity + rightWheelVelocity) / 2
        dt = currentTimestamp - self.timestamp
        # XXX: May need to change this line 
        if abs(v) > minSpeed:
            w = yawRate   #(yawRate+0.011)*0.98
        else:
            w = 0.0
        
        x = self.x + v*cos(self.theta) * dt
        y = self.y + v*sin(self.theta) * dt
        theta = self.theta + w * dt

        return x, y, theta
        
    @staticmethod
    def interpolate (pose1, pose2, ratio):
        if (pose1.timestamp > pose2.timestamp) :
            raise ValueError ("pose1 timestamp must be > pose2")
        td = (pose2.timestamp - pose1.timestamp)
        intpose = Pose(pose1.timestamp + ratio*td, 
            pose1.x + ratio*(pose2.x-pose1.x),
            pose1.y + ratio*(pose2.y-pose1.y),
            pose1.z + ratio*(pose2.z-pose1.z))
        q1 = pose1.quaternion()
        q2 = pose2.quaternion()
        qInt = trafo.quaternion_slerp(q1, q2, ratio)
        intpose.qx, intpose.qy, intpose.qz, intpose.qw = \
            qInt[0], qInt[1], qInt[2], qInt[3]
        return intpose
        
    @staticmethod
    def average (*poses):
        avgpose = Pose()
        xs = [p.x for p in poses]
        ys = [p.y for p in poses]
        zs = [p.z for p in poses]
        avgpose.x = sum(xs) / len(poses)
        avgpose.y = sum(ys) / len(poses)
        avgpose.z = sum(zs) / len(poses)
        avgpose.timestamp = np.average([p.timestamp for p in poses])
        return avgpose
        
    def publish (self, tfBroadCaster, frame1, frame2):
        tfBroadCaster.sendTransform(
            (self.x, self.y, self.z),
            (self.qx, self.qy, self.qz, self.qw),
            rospy.Time.from_sec(self.timestamp),
            frame1, frame2
        )

    # Output euler angle in order of: Roll, Pitch, Yaw
    def euler (self):
        return np.array(trafo.euler_from_quaternion([self.qx, self.qy, self.qz, self.qw]))
        
    def setRPY (self, roll, pitch, yaw):
        pass

    def distance (self, pose):
        return np.linalg.norm([self.x-pose.x, self.y-pose.y, self.z-pose.z])
    
    def inverse (self):
        (qx, qy, qz, qw) = np.array([-self.qx, -self.qy, -self.qz, self.qw]) / np.linalg.norm([self.qx, self.qy, self.qz, self.qw])
        return Pose(self.timestamp, -self.x, -self.y, -self.z, qx, qy, qz, qw)
    
    def toMat4 (self):
        mat4 = np.eye(4)
        rotm = trafo.quaternion_matrix([self.qx, self.qy, self.qz, self.qw])
        mat4[0:3,3] = (self.x, self.y, self.z)
        mat4[0:3, 0:3] = rotm
        return mat4
    
    def toRotMat (self):
        return trafo.quaternion_matrix([self.qx, self.qy, self.qz, self.qw])
    
    def __mul__ (self, posev):
        p = posev.apply(self)
        p.timestamp = self.timestamp
        return p
    
    def __rmul__ (self, posev):
        return self.apply(posev)
    
#     ot * self
    def apply (self, ot):
        rotmat1 = self.toRotMat()
        rotmat2 = ot.toRotMat()
        q = trafo.quaternion_from_matrix(rotmat1.dot(rotmat2))
        return Pose (self.timestamp,
                     rotmat1[0][0:3].dot([ot.x, ot.y, ot.z]) + self.x,
                     rotmat1[1][0:3].dot([ot.x, ot.y, ot.z]) + self.y,
                     rotmat1[2][0:3].dot([ot.x, ot.y, ot.z]) + self.z,
                     q[0], q[1], q[2], q[3])
        
    def doApplyMe (self, ot):
        p = self.apply(ot)
        self.x = p.x
        self.y = p.y
        self.z = p.z
        self.qx = p.qx
        self.qy = p.qy
        self.qz = p.qz
        self.qw = p.qw
        
    def measureErrorLateral (self, groundTruth, timeTolerance=0.1, useZ=False):
        def doMeasureDistance (p, q, useZ):
            if useZ :
                return np.linalg.norm ([p.x-q.x, p.y-q.y, p.z-q.z], 2)
            else :
                return np.linalg.norm ([p.x-q.x, p.y-q.y], 2)
        
        pMin, pMax = groundTruth.findNearPosesByTime (self, timeTolerance)
        # XXX: I know this is Wrong
#         if pMin is None or pMax is None:
#             return 1000
        if pMax is None:
            return doMeasureDistance(self, pMin, useZ)
        if pMin is None:
            return doMeasureDistance(self, pMax, useZ)
#         if (pMin is None) or (pMax is None) :
#             return -2.0

        pointChk, c = ClosestPointInLine(pMin, pMax, self, True)
        
        # Ideal case
        if c>=0.0 and c<=1.0:
            return doMeasureDistance(pointChk, self, useZ)
        
        # Bad case
        elif c<0.0:
            return doMeasureDistance(pMin, self, useZ)
#             return -3.0
        else:
            return doMeasureDistance(pMax, self, useZ)
#             return -4.0


class PoseTable :
    def __init__ (self):
        self.table = []
        self.idList = {}
        self.c = 0
    
    def __setitem__ (self, key, value):
        self.table.append (value)
        self.idList[key] = self.c
        self.c += 1
        
    def __getitem__ (self, key):
        p = self.idList[key]
        return self.table[p]
    
    def __len__ (self):
        return len(self.table)
    
    def __iadd__ (self, offset):
        for pose in self.table :
            pose.offsetTime (offset)
        return self
    
    def __isub__ (self, offset):
        return self.__iadd__(-offset)
        
    def append (self, pose):
        self.table.append (pose)
        ckeys = self.idList.keys()
        if (len(ckeys)==0):
            ckey = -1
        else :
            ckey = max (ckeys)
        self.idList[ckey+1] = self.c
        self.c += 1
        
    def apply (self, poseX):
        for pose in self.table:
            pose = pose.doApplyMe (poseX)
        pass
        
    def length (self, tolerance=0):
        """
        Compute distance spanned by this pose. If miliSecTolerance is not specified, \
        we assume that there is no gap
        """
        totaldist = 0
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            if (tolerance>0) :
                if abs(ppose.timestamp - cpose.timestamp) > tolerance:
                    print('far')
                    continue
            dist = np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z])
            totaldist += dist
        return totaldist
        
    def lengths (self):
        dists = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            dists.append (np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z]))
        return dists
        
    def timeLengths (self):
        timeDists = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            timeDists.append (abs(cpose.timestamp - ppose.timestamp))
        return timeDists        
        
    
    def toArray (self, includeTimestamp=False):
        if (includeTimestamp==True) :
            mlt = [[p.timestamp, p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw] for p in self.table]
        else :
            mlt = [[p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw] for p in self.table]
        return np.array(mlt)
    
    def findNearestByTime (self, pose, tolerance=0):
        if (pose.timestamp < self.table[0].timestamp) :
            raise KeyError ("Timestamp less than table")
        if (pose.timestamp > self.table[self.c-1].timestamp) :
            raise KeyError ("Timestamp is outside table: " + str(pose.timestamp))
        candidates = set()
        i = 0
        for p in range(len(self.table)) :
            i = p
            cpose = self.table[i]
            if (cpose.timestamp > pose.timestamp) :
                candidates.add(cpose)
                i-=1
                break
        while i!=0 :
            cpose = self.table[i]
            i -= 1
            candidates.add (cpose)
            if (cpose.timestamp < pose.timestamp) :
                candidates.add (cpose)
                break
        if (tolerance>0) :
            tcandidates=[]
            for c in candidates:
                c.tdif = abs(c.timestamp-pose.timestamp)
                if c.tdif > tolerance:
                    pass
                else:
                    tcandidates.append(c)
            return sorted (tcandidates, key=lambda pose: pose.tdif)
        #return sorted (candidates, key=lambda pose: pose.timestamp)
        return min(candidates, key=lambda p: abs(p.timestamp-pose.timestamp))
 
 
    def findNearPosesByTime (self, srcpose, tolerance=0.1):
        if (srcpose.timestamp < self.table[0].timestamp) :
            raise KeyError ("Timestamp less than table")
        if (srcpose.timestamp > self.table[-1].timestamp) :
            raise KeyError ("Timestamp is outside table: " + str(srcpose.timestamp))
 
        nearMin = None
        nearMax = None
        for p in range(len(self)):
            i = p
            cpose = self.table[i]
            if (cpose.timestamp > srcpose.timestamp):
                nearMax = copy(cpose)
                break
        while i != 0 :
            cpose = self.table[i]
            i -= 1
            if (cpose.timestamp < srcpose.timestamp):
                nearMin = copy (cpose)
                break
        return (nearMin, nearMax)
    
    
    def interpolateByTime (self, srcpose, tolerance=0.1):
        pmin, pmax = self.findNearPosesByTime (srcpose, tolerance)
        tRatio = (srcpose.timestamp - pmin.timestamp) / (pmax.timestamp - pmin.timestamp)
        
        # Interpolation of Position
        pvmin = pmin.coord()
        pvmax = pmax.coord()
        posInt = pvmin + tRatio * (pvmax - pvmin)
        
        # Interpolation of orientation
        qmin = pmin.quaternion()
        qmax = pmax.quaternion()
        qInt = trafo.quaternion_slerp(qmin, qmax, tRatio)
        return Pose(srcpose.timestamp, posInt[0], posInt[1], posInt[2], \
            qInt[0], qInt[1], qInt[2], qInt[3])
        
    
    def interpolateByProjection (self, srcpose, tolerance=0.1):
        pmin, pmax = self.findNearPosesByTime (srcpose, tolerance)
        
    
    def findNearestInTime (self, timestamp, tolerance=0.1):
        candidates = set()
        for p in self.table:
            tdiff = abs(p.timestamp - timestamp)
            if (tdiff < tolerance):
                candidates.add(p)
            if p.timestamp > timestamp:
                break
        if (len(candidates)==0):
            return None
        return min(candidates, key=lambda p: abs(p.timestamp-timestamp))
            
    def findNearestByDistance (self, pose, returnIdx=False, *args):
        if (returnIdx==False):
            return min(self.table, 
                       key=lambda p: 
                           np.linalg.norm(pose.coord()-p.coord()))
            
        elif (returnIdx==True) :
            dist = np.array([np.linalg.norm(pose.coord()-p.coord()) for p in self.table])
            return np.argmin(dist)
        
        elif len(args)>0 :
            posecoord = np.array([pose, returnIdx, args[0]])
            return min(self.table, 
                key=lambda p: 
                    np.linalg.norm(posecoord-p.coord()))
            
        
    def last(self):
        return self.table[-1]
    
    @staticmethod
    def plotMulti (*tables):
        pass
    
    def plot (self, col1, col2, **kwargs):
        array = self.toArray()
        return plt.plot(array[:,col1], array[:,col2], **kwargs)
        
    def plotRange (self, col1, col2, rangeFrom, rangeTo, **kwargs):
        array = self.toArray()
        return plt.plot(array[rangeFrom:rangeTo, col1], array[rangeFrom:rangeTo, col2], **kwargs)
    
    # Choosing columns: 1->X, 2->Y, 3->Z
    def plotTimeToAxis (self, col):
        matr = self.toArray(True)
        return plt.plot(matr[:,0], matr[:,col])
    
    @staticmethod 
    def loadCsv (filename):
        mat = np.loadtxt(filename)
        records = PoseTable ()
        for r in mat :
            p = Pose (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7])
            records.append (p)
        return records
    
    @staticmethod
    def loadFromBagFile (filename, sourceFrameName=None, targetFrameName=None) :
        bagsrc = rosbag.Bag(filename, mode='r')
        #topicInfo = bagsrc.get_type_and_topic_info('/tf')
        i = 0
        bagRecord = PoseTable ()
        # Message timestamp is what recorded by `rosbag record'
        # Transform timestamp is what reported by publisher
        for topic, msg, msgTimestamp in bagsrc.read_messages('/tf') :
            transform = msg.transforms[0].transform
            header = msg.transforms[0].header
            tfTimestamp = header.stamp
            child_frame_id = msg.transforms[0].child_frame_id
            if (sourceFrameName!=None and targetFrameName!=None) :
                if header.frame_id!=sourceFrameName or child_frame_id!=targetFrameName :
                    continue
            pose = Pose (tfTimestamp.to_sec(), 
                transform.translation.x, transform.translation.y, transform.translation.z,
                transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
            pose.counter = i
            pose.msgTimestamp = msgTimestamp.to_sec()
            bagRecord[i] = pose 
            i += 1
        return bagRecord
        
    @staticmethod
    def loadFromArray (msrc):
        if (msrc.shape[1] != 8):
            raise ValueError ('Source has no timestamp')
        # try to sort
        msrc = sorted(msrc, key=lambda r: r[0])
        table = PoseTable ()
        for row in msrc:
            pose = Pose(row)
            table.append(pose)       
        return table

    @staticmethod
    def getFrameList (filename):
        bagsrc = rosbag.Bag(filename, mode='r')
        frames = {}
        for topic, msg, timestamp in bagsrc.read_messages('/tf'):
            
            transform = msg.transforms[0].transform
            header = msg.transforms[0].header
            child_frame_id = msg.transforms[0].child_frame_id
            transformFrame = {'from':header.frame_id, 'to':child_frame_id, 'start':timestamp.to_sec()}
            
            key = header.frame_id+child_frame_id
            if (key not in frames):
                frames[key] = transformFrame
                
        return frames.values()
        
    # Find all poses in current table that are in range of targetPoses timeframe
    def getAllInTimeRanges (self, targetPoses):
        matchInTime = PoseTable()
        
        p1 = targetPoses[0]
        p2 = targetPoses.last()
        
        for p in self.table:
            if p.timestamp >= p1.timestamp and p.timestamp<=p2.timestamp:
                matchInTime.append (copy (p))
        
        return matchInTime
        
    @staticmethod
    def compareErrorsByTime (poseTbl1, poseTbl2, useZ=True):
        """
        poseTbl1 -> for source table
        poseTbl2 -> for ground truth
        """
        errorVect = []
        i=0
        for pose in poseTbl1.table:
            try:
                nearp = poseTbl2.findNearestByTime(pose)
                if (useZ):
                    errv = np.linalg.norm([pose.x-nearp.x, pose.y-nearp.y, pose.z-nearp.z], 2)
                else:
                    errv = np.linalg.norm([pose.x-nearp.x, pose.y-nearp.y], 2)
                errorVect.append([pose.timestamp, errv])
                i+=1
                #if i>=10000:
                #    break
                print ("{} out of {}".format(i, len(poseTbl1)))
            except KeyError as e:
                print e
        return errorVect
        

    @staticmethod
    def compareErrorsByDistance (poseTblSource, groundTruth, useZ=True):
        errorVect = []
        i=0
        for pose in poseTblSource.table:
            try:
                nearp = groundTruth.findNearestByDistance(pose)
                if (useZ):
                    errv = np.linalg.norm([pose.x-nearp.x, pose.y-nearp.y, pose.z-nearp.z], 2)
                else:
                    errv = np.linalg.norm([pose.x-nearp.x, pose.y-nearp.y], 2)
                errorVect.append([pose.timestamp, errv])
                i+=1
                #if i>=10000:
                #    break
                print ("{} out of {}".format(i, len(poseTblSource)))
            except KeyError as e:
                print e
        return errorVect

    @staticmethod
    def compareLateralErrors (poseTblSource, groundTruth, tolerance=0.15, useZ=False):
        i = 0
        for pose in poseTblSource.table:
            try:
                errMeas = pose.measureErrorLateral (groundTruth, tolerance, useZ)
                pose.measuredError = errMeas
#                 errorVect.append ([pose.timestamp, errMeas])
            except KeyError as e:
                pose.measuredError = -1.0
            i += 1
            print ("{} out of {}".format(i, len(poseTblSource)))


    @staticmethod
    # XXX: Unfinished
    def removeSpuriousPoints (poseTbl1):
        newposetbl = PoseTable()
        for pose in poseTbl1.table:
            pass
        return newposetbl
        
    def findBlankTime(self, timeTolerance=0.5):
        blanks = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            if abs(cpose.timestamp - ppose.timestamp) > timeTolerance:
                blanks.append([ppose, cpose])
        return blanks
        
    def lengthFrom2Pose (self, poseIndex1, poseIndex2):
        if (type(poseIndex1)==int):
            dist = 0.0
            for i in range(poseIndex1+1, poseIndex2+1):
                cpose = self.table[i]
                ppose = self.table[i-1]
                cdist = np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z], 2)
                dist += cdist
            return dist
    
    def lengthFrom2Times(self, time1, time2):
        pose1 = self.findNearestInTime(time1, 0.25)
        idx1 = self.table.index(pose1)
        pose2 = self.findNearestInTime(time2, 0.25)
        idx2 = self.table.index(pose2)
        return self.lengthFrom2Pose (idx1, idx2)
        
    def subset (self, startIdx, stopIdx):
        poseTblSubset = PoseTable()
        for i in range(startIdx, stopIdx+1):
            p = self.table[i]
            poseTblSubset.append(p)
        return poseTblSubset
        
    def transform (self, dpose):
        pass
        
    def findBlankLengthFromGroundTruth (self, groundTruthTbl):
        tolerance = 0.25
        blankDistFront = 0
        # Find blank distance in front
        if groundTruthTbl[0].timestamp < self.table[0].timestamp:
            pgrnd = groundTruthTbl.findNearestInTime (self.table[0].timestamp, tolerance)
            idx = groundTruthTbl.table.index(pgrnd)
            blankDistFront = groundTruthTbl.lengthFrom2Pose (0, idx)
        else:
            blankDistFront = 0
        # Find blank distance in rear
        blankDistRear = 0
        if (groundTruthTbl.last().timestamp > self.table[-1].timestamp):
            pgrnd = groundTruthTbl.findNearestInTime (self.table[-1].timestamp, tolerance)
            idx = groundTruthTbl.table.index (pgrnd)
            blankDistRear = groundTruthTbl.lengthFrom2Pose (idx, len(groundTruthTbl)-1)
        else:
            blankDistRear = 0
        # Find blank distances in middle
        blankPoses = self.findBlankTime(tolerance)
        blankDistMid = 0
        for bPose in blankPoses:
            d = groundTruthTbl.lengthFrom2Times (bPose[0].timestamp, bPose[1].timestamp)
            blankDistMid += d
        return blankDistFront + blankDistMid + blankDistRear
        

    def saveToBag (self, bagFileName, parentFrame, childFrame, append=False):
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import Header
        from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
        import rosbag
        
        def create_tf_message (pose):
            header = Header (stamp=rospy.Time(pose.timestamp))
            tfmsg = TFMessage(transforms = [TransformStamped()])
            tfmsg.transforms[0].header = copy(header)
            tfmsg.transforms[0].header.frame_id = parentFrame
            tfmsg.transforms[0].child_frame_id = childFrame
            tfmsg.transforms[0].transform = Transform()
            tfmsg.transforms[0].transform.translation = Vector3(pose.x, pose.y, pose.z)
            tfmsg.transforms[0].transform.rotation = Quaternion(x=pose.qx, \
                y=pose.qy, 
                z=pose.qz, 
                w=pose.qw)
            return tfmsg

        bagfile = None
        if (append==False):
            bagfile = rosbag.Bag(bagFileName, mode='w')
        else :
            bagfile = rosbag.Bag(bagFileName, mode='a')
        i = 0
        for pose in self.table:
            tfmsg = create_tf_message(pose)
            bagfile.write('/tf', tfmsg, t=rospy.Time.from_sec(pose.msgTimestamp))
            print ("{} / {}".format(i, len(self.table)))
            i+=1
        bagfile.close()
            
            
    @staticmethod
    def loadSegwayStatusFromBag (bagFilename, limitMsg=0) :
        segwayPose = PoseTable()
        
        bagsrc = rosbag.Bag(bagFilename, mode='r')
        
        cPose = Pose()
        cPose.theta = 0.0
        i = 0
        
        for topic, msg, timestamp in bagsrc.read_messages():
            try:
                if cPose.timestamp == 0:
                    cPose.timestamp = timestamp.to_sec()
                    continue
                x, y, theta = cPose.segwayMove(timestamp.to_sec(), 
                    msg.segway.left_wheel_velocity, 
                    msg.segway.right_wheel_velocity, 
                    msg.segway.yaw_rate)
                cPose.x = x
                cPose.y = y
                cPose.theta = theta
                cPose.timestamp = timestamp.to_sec()
                
                segwayPose.append (copy(cPose))
                i += 1
                if (limitMsg!=0 and i>=limitMsg):
                    break
                print (i)
                
            except KeyError:
                continue
        
        return segwayPose
    
    
    def increaseTimeResolution (self, numToAdd=10):
        NewPoseTable = PoseTable()
        for i in range(len(self)-1) :
            p1 = self[i]
            p2 = self[i+1]
            NewPoseTable.append(p1)
            
            rt = 1.0 / float(numToAdd)
            j = 0.0
            while (j < 1.0) :
                j += rt
                pnew = Pose.interpolate(p1, p2, j)
                NewPoseTable.append(pnew)
            
            NewPoseTable.append(p2)
        
        return NewPoseTable
        
        
    
    
def joinPoseTables (*poseTbls):
    #Find maximum & minimum time
    mintimes = [ptb[0].timestamp for ptb in poseTbls]
    startTime = min(mintimes)
    maxtimes = [ptb.last().timestamp for ptb in poseTbls]
    stopTime = max(maxtimes)
    
    # Find optimal time resolution
    def timeDiffs (poseTbl) :
        diff = []
        for p in range(1, len(poseTbl.table)):
            cpose = poseTbl.table[p]
            ppose = poseTbl.table[p-1]
            diff.append(cpose.timestamp - ppose.timestamp)
        return diff
    poseTblDiffs = [timeDiffs(ptbl) for ptbl in poseTbls]
    minDiffs = [min(td) for td in poseTblDiffs]
    timeRez = min(minDiffs)

    poseList = set()
    for ptbl in poseTbls:
        for pose in ptbl.table:
            pose.parent = ptbl
            poseList.add(pose)
    allPosesList = sorted(poseList, key=lambda p: p.timestamp)

    jointPoses = PoseTable()    
    for pose in allPosesList:
        if pose not in poseList:
            continue
        cPoses = []
        cPoses.append(pose)
        for ptbl in poseTbls:
            if (pose.parent==ptbl):
                continue
            friend = ptbl.findNearestInTime(pose.timestamp, 2*timeRez)
            if friend != None and friend in poseList:
                cPoses.append (friend)
        poseAvg = Pose.average(*cPoses)
        jointPoses.append(poseAvg)
        for p in cPoses:
            poseList.discard(p)
        # For debugging progress
        print ("Length: {} / {}".format(len(jointPoses), len(allPosesList)))
    return jointPoses        


def OrbFixOffline (orbLocalisationBagFilename, mapCsv):
    offset = 1
    orbLoc = PoseTable.loadFromBagFile(orbLocalisationBagFilename, 'ORB_SLAM/World', 'ORB_SLAM/Camera')
    mapArray = np.loadtxt(mapCsv)
    orbMapTbl = np.array([[r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7]] for r in mapArray])
    orbMap = PoseTable.loadFromArray(orbMapTbl)
    ndtMapTbl = np.array([[r[0],r[8],r[9],r[10],r[11],r[12],r[13],r[14]] for r in mapArray])
    ndtMap = PoseTable.loadFromArray(ndtMapTbl)
    
    for loc in orbLoc.table:
        loc.kfId = orbMap.findNearestByDistance(loc, True)
        loc.kf = orbMap[loc.kfId]

    # Fix axes
    for pose in orbLoc.table:
        x=pose.x
        y=pose.y
        z=pose.z
        pose.x=z
        pose.y=-x
        pose.z=-y
        x=pose.kf.x
        y=pose.kf.y
        z=pose.kf.z
        pose.kf.x=z
        pose.kf.y=-x
        pose.kf.z=-y
        
        ndtPose = ndtMap[pose.kfId]
        ndtPoseOffset = None
        try:
            ndtPoseOffset = ndtMap[pose.kfId-offset]
            kfOffset = orbMap[pose.kfId-offset]
        except KeyError:
            continue
        
        scale = np.linalg.norm(ndtPose.coord()-ndtPoseOffset.coord()) / \
            np.linalg.norm(pose.kf.coord()-kfOffset.coord())
        poseRel = Pose(0, ndtPose.x-pose.x, ndtPose.y-pose.y, ndtPose.z-pose.z)
            
        pose.cx = ndtPose.x + scale*poseRel.x
        pose.cy = ndtPose.y + scale*poseRel.y
        pose.cz = ndtPose.z + scale.poseRel.z

    return orbLoc, orbMap, ndtMap
    


# Custom bag reader class
class BagReader (rosbag.Bag):
    
    def __init__ (self, bagpath, topicname=None):
        super(BagReader, self).__init__(bagpath, 'r')
        self.rTopicName = topicname
        
    def readByTime (self, second):
        rtm = rospy.Time.from_sec(second)
        if self.rTopicName is not None:
            for topic,msg,time in self.read_messages(topics=self.rTopicName, start_time=rtm):
                if msg is None:
                    raise KeyError("Message not found in that time")
                return copy(msg)
    
    def readByCount (self, counter):
        pass


def formatResultAsRecords (resultMat):
    records = PoseTable()
    for r in range(len(resultMat)) :
        id = int (resultMat[r][0])
        pose = Pose(resultMat[r][1:])
        records[id] = pose
    return records


def flipOrbToNdt (orbPose):
    qOrb = [orbPose.qx, orbPose.qy, orbPose.qz, orbPose.qw]
    orbFlip = trafo.concatenate_matrices(
        trafo.quaternion_matrix(qOrb),
        trafo.rotation_matrix(np.pi/2, (1,0,0)),
        trafo.rotation_matrix(np.pi/2, (0,0,1))
    )
    return trafo.quaternion_from_matrix(orbFlip)


def readMessage (bag, topic, timestamp):
    tm = rospy.Time.from_sec(timestamp)
    for topic, msg, time in bag.read_messages(topics=topic, start_time=tm):
        return msg


if __name__ == '__main__' :
    imageList = BagReader ('/home/sujiwo/Data/Road_Datasets/2016-02-05-13-32-06/localizationResults/map2/resultz.bag', '/orbslamdebug/compressed')
    msg = imageList.readByTime(1454648393.2110474)
#     testmap1 = PoseTable.loadFromBagFile('/home/sujiwo/Data/Road_Datasets/2016-02-05-13-32-06/localizationResults/map1/slowrate.bag', '/ORB_SLAM/World', '/ORB_SLAM/ExtCamera')
#     testpose = testmap1[19333]
#     erz = testpose.measureErrorLateral (groundTruth, timeTolerance=0.25)
    
    pass
    