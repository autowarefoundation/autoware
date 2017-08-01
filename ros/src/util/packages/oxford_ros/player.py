#!/usr/bin/python

import os
import sdk
import time
import rospy
import yaml
import cv2
import cv_bridge
import rospkg
from tf import transformations
from tf import TransformBroadcaster
import numpy as np
from sdk.nonblocking import NonblockingKeybInput
from sdk.FileCollector import FileCollector
from sdk.TimerProcess import TimerProcess

from geometry_msgs.msg import PoseStamped as PoseMsg
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from rosgraph_msgs.msg import Clock
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import std_msgs
from rospkg.common import ResourceNotFound



class ImagePlayer:
    
    frameId = 'camera_view'
    topicName = '/oxford/image_color'
    
    def __init__ (self, dataset, _publish=True, raw=False):
        self.raw = raw
        self.publish = _publish
        self.firstValidId = -1
        
        if (self.publish):
            if (self.raw):
                self.publisher = rospy.Publisher ('/oxford/image_raw', ImageMsg, queue_size=1)
            else:
                self.publisher = rospy.Publisher ('/oxford/image_color', ImageMsg, queue_size=1)
            self.publisherImgInfo = rospy.Publisher('/oxford/camera_info', CameraInfo, queue_size=1)
        
        self.imageList = dataset.getStereo()
        pkgpack = rospkg.RosPack()
        try:
            path = pkgpack.get_path('oxford_ros')
        except ResourceNotFound:
            path = os.path.dirname(os.path.abspath(__file__)) 
        self.cameraModel = sdk.CameraModel (path+'/models', sdk.CameraModel.cam_stereo_center)
        
        calib_file = file(path+'/calibration_files/bb_xb3_center.yaml')
        conf = yaml.load(calib_file)
        self.imageShape = (conf['image_height'], conf['image_width'])
        self.camera_matrix2 = np.reshape(conf['camera_matrix']['data'], (3,3))
        self.camera_matrix = np.eye(3)
        self.camera_matrix[0,0] = self.cameraModel.focal_length[0]
        self.camera_matrix[1,1] = self.cameraModel.focal_length[1]
        self.camera_matrix[0,2] = self.cameraModel.principal_point[0]
        self.camera_matrix[1,2] = self.cameraModel.principal_point[1]
        
#         self.projection_matrix = np.reshape(conf['projection_matrix']['data'], (3,4))
        self.projection_matrix = np.zeros((3,4))
        self.projection_matrix[0:3,0:3] = self.camera_matrix

        self.distortion_coefs = np.array(conf['distortion_coefficients']['data'])
        
#         self.calibrator = cv2.cv.Load(path+'/calibration_files/bb_xb3_center.yaml')
        self.cvbridge = cv_bridge.CvBridge()
        self.cameraInfo = self.createCameraInfoMessage()
        
    def createCameraInfoMessage (self):
        cameraInfo = CameraInfo()
        cameraInfo.header.frame_id = self.frameId
        cameraInfo.width = self.imageShape[1]
        cameraInfo.height = self.imageShape[0]
        cameraInfo.K = self.camera_matrix.reshape((1,9))[0]
        cameraInfo.P = self.projection_matrix.reshape((1,12))[0]
        cameraInfo.distortion_model = 'plumb_bob'
        cameraInfo.D = self.distortion_coefs
        return cameraInfo
        
        
    def initializeRun (self):
#        File Collector
        fileList = [
            self.imageList[pr]['center'] 
            for pr in range(len(self.imageList)) 
                if pr>=self.firstValidId
        ]
        self.collector = FileCollector(fileList, self.readFileFunc)
        
    def close (self):
        print ("Closing images")
        self.collector.close()

    def _getEvents (self):
        eventList = [ 
            {'timestamp':self.imageList[i]['timestamp'], 'id':i} 
                for i in range(len(self.imageList)) 
        ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        image_ctr = self.collector.pick()
        if (image_ctr is None):
            return
        
        msg = self.createMessageFromMat(image_ctr, timestamp)
        self.cameraInfo.header.stamp = msg.header.stamp
        if (publish):
            self.publisher.publish(msg)
            self.publisherImgInfo.publish(self.cameraInfo)
        else:
            return msg
        
    def createMessageFromMat (self, imgMat, timestamp, compressed=False):
        if self.raw==False:
            if compressed:
                msg = self.cvbridge.cv2_to_compressed_imgmsg(imgMat, dst_format='png')
            else:
                msg = self.cvbridge.cv2_to_imgmsg(imgMat, 'bgr8')
        else:
            msg = self.cvbridge.cv2_to_imgmsg(imgMat, 'bayer_gbrg8')
        msg.header.stamp = rospy.Time.from_sec(timestamp)
        msg.header.frame_id = self.frameId
        return msg
        
    def imagePostProcessing (self, imageMat):
        if (self.raw==False):
            imageMat = cv2.cvtColor(imageMat, cv2.COLOR_BAYER_GR2BGR)
            imageMat = cv2.undistort(imageMat, self.camera_matrix2, self.distortion_coefs)
        return imageMat
    
    def readFileFunc (self, path):
        image = cv2.imread(path, cv2.IMREAD_ANYCOLOR)
        if (image is None):
            print ("Image is empty: {}".format(path))
            return None
        return self.imagePostProcessing(image)

    @staticmethod
    def iteratorToMessage (dataset, compressed=False):
        player = ImagePlayer (dataset, _publish=False)
        for imgInfo in player.imageList:
            path = imgInfo['center']
            timestamp = imgInfo['timestamp']
            yield player.createMessageFromMat(player.readFileFunc (path), timestamp, compressed)



class Lidar3Player:
    _lidarName = 'ldmrs'
    topicName = '/oxford/ldmrs'
    
    def __init__ (self, dataset, publish=True):
        self.firstValidId = -1
        self.lidarFileSet = dataset.getMainLidar()
        if (publish):
            self.publisher = rospy.Publisher (self.topicName, PointCloud2, queue_size=10)
        else:
            self.publisher = None
    
    def close (self):
        print ("Closing {} set".format(self._lidarName))
        self.collector.close()
    
    def initializeRun (self):
        lidarFileList = [
            self.lidarFileSet[p]['path']
            for p in range(len(self.lidarFileSet))
                if p >= self.firstValidId
        ]
        self.collector = FileCollector(lidarFileList, self.readFileFunc)
    
    def _getEvents (self):
        eventList = [ {
            'timestamp': self.lidarFileSet[i]['timestamp'],
            'id': i
        } for i in range(len(self.lidarFileSet)) ]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        scan = self.collector.pick()
        header = std_msgs.msg.Header(
            stamp=rospy.Time.from_sec(timestamp), 
            frame_id=Lidar3Player._lidarName)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=16, datatype=PointField.FLOAT32, count=1)
        ]
        msg = pcl2.create_cloud(header, fields, scan)
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
    def readFileFunc (self, path):
        scan = np.fromfile(path, np.double)
        return scan.reshape ((len(scan) // 3,3)).astype(np.float32)
    
    @staticmethod
    def iteratorToMessage (dataset):
        player = Lidar3Player (dataset, publish=False)
        for scanr in player.lidarFileSet :
            timestamp = scanr['timestamp']
            path = scanr['path']
            scan = player.readFileFunc(path)
            header = std_msgs.msg.Header(
                stamp=rospy.Time.from_sec(timestamp), 
                frame_id=Lidar3Player._lidarName)
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=16, datatype=PointField.FLOAT32, count=1)
            ]
            yield pcl2.create_cloud(header, fields, scan)
                    

class Lidar2Player (Lidar3Player):
    _lidarName = 'lms_front'
    topicName = '/oxford/lms_front'
    
    def __init__ (self, dataset, _publish=True):
        self.firstValidId = -1
        self.lidarFileSet = dataset.getLidar2D('front')
        if (_publish):
            self.publisher = rospy.Publisher (self.topicName, PointCloud2, queue_size=10)
    
    def _passEvent (self, timestamp, eventId, publish=True):
        scan = self.collector.pick()
        msg = Lidar2Player.create2DScanMessage(scan, timestamp)
        if (publish):
            self.publisher.publish(msg)
        else:
            return msg
        
    @staticmethod
    def create2DScanMessage (scan, timestamp):
        scanz = np.zeros((scan.shape[0], 4), dtype=scan.dtype)
        scanz[:,0:2] = scan[:,0:2]
        # X Axis from scans is negated to comform with right-hand convention
        scanz[:,0] = -scanz[:,0]
        scanz[:,3] = scan[:,2]
#         scan = scan[:,0:2]
        header = std_msgs.msg.Header(
            stamp=rospy.Time.from_sec(timestamp), 
            frame_id=Lidar2Player._lidarName)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='i', offset=24, datatype=PointField.FLOAT32, count=1)
        ]
        return pcl2.create_cloud(header, fields, scanz)


    @staticmethod
    def iteratorToMessage(dataset):
        player = Lidar2Player(dataset)
        for r in player.lidarFileSet:
            scan = player.readFileFunc(r['path'])
            yield Lidar2Player.create2DScanMessage(scan, r['timestamp'])


class PosePlayer:
    topicName = '/oxford/pose'
    
    def __init__ (self, dataset):
        self.firstValidId = -1
        self.poses = dataset.getIns()
        self.publisher = rospy.Publisher (self.topicName, PoseMsg, queue_size=1)
        self.tfb = TransformBroadcaster()
        
    def close(self):
        pass
    
    def initializeRun (self):
        pass
    
    def _getEvents (self):
        eventList = [{'timestamp':self.poses[p,0], 'id':p} for p in range(len(self.poses))]
        return eventList
    
    def _passEvent (self, timestamp, eventId, publish=True):
        poseRow = self.poses[eventId]
        curPose = PosePlayer.createPoseFromRPY(
            poseRow[1], poseRow[2], poseRow[3], poseRow[4], poseRow[5], poseRow[6])
        curPose.header.stamp = rospy.Time.from_sec(timestamp)
        curPose.header.frame_id = 'world'
        if (publish):
            self.publisher.publish(curPose)
            self.tfb.sendTransform(
                (curPose.pose.position.x,
                 curPose.pose.position.y,
                 curPose.pose.position.z),
                (curPose.pose.orientation.x,
                 curPose.pose.orientation.y,
                 curPose.pose.orientation.z,
                 curPose.pose.orientation.w),
                rospy.Time.from_sec(timestamp),
                'base_link',
                'world'
            )
        else:
            return curPose
        
    @staticmethod
    def createPoseFromRPY (x, y, z, roll, pitch, yaw):
        p = PoseMsg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        qt = transformations.quaternion_from_euler(roll, pitch, yaw)
        p.pose.orientation.x = qt[0]
        p.pose.orientation.y = qt[1]
        p.pose.orientation.z = qt[2]
        p.pose.orientation.w = qt[3]
        return p
    
    @staticmethod
    def createPoseMessages (dataset):
        from tf2_msgs.msg import TFMessage
        from std_msgs.msg import Header
        from geometry_msgs.msg import Transform, \
            TransformStamped, Vector3, Quaternion
        
        poseMsgs = []
        poseList = dataset.getIns()
        for ps in poseList :
            curPose = PosePlayer.createPoseFromRPY(
                ps[1], ps[2], ps[3], ps[4], ps[5], ps[6])
            curPose.header.stamp = rospy.Time.from_sec(ps[0])
            curPose.header.frame_id = 'world'
            # create TF Message
            tfmsg = TFMessage(transforms=[TransformStamped()])
            tfmsg.transforms[0].header = Header(stamp=rospy.Time.from_sec(ps[0]))
            tfmsg.transforms[0].header.frame_id = 'world'
            tfmsg.transforms[0].child_frame_id = 'base_link'
            tfmsg.transforms[0].transform = Transform()
            tfmsg.transforms[0].transform.translation = \
                Vector3(curPose.pose.position.x,
                    curPose.pose.position.y,
                    curPose.pose.position.z)
            tfmsg.transforms[0].transform.rotation = \
                Quaternion(x=curPose.pose.orientation.x,
                    y=curPose.pose.orientation.y,
                    z=curPose.pose.orientation.z,
                    w=curPose.pose.orientation.w)
            poseMsgs.append({'geom':curPose, 'tf':tfmsg})
        return poseMsgs
            


class PlayerControl:
    def __init__ (self, datadir, rate=1.0, start=0.0):
        self.rate = float(rate)
        self.eventList = []
        self.dataset = sdk.Dataset(datadir)
        self.players = []
        self.startTime = start
        
    def add_data_player (self, _dataPlayer):
        if (_dataPlayer is None):
            return
        self.players.append(_dataPlayer)
    
    def run (self):
        self.initRun()
        isPause = NonblockingKeybInput()
        
        while (True):
            # Wait for a timer event
            self.timer.eventNotification.wait()
            self.timer.eventNotification.clear()
            curEvent = self.eventList[self.timer.currentEventTimerId]
#             ct = curEvent['timestamp']
            ct = self.timer.currentTimestamp
            curEvent['object']._passEvent (ct, curEvent['id'])
            
            if (rospy.is_shutdown()):
                break
            if isPause.spacePressed():
                self.timer.pause()
                isPause.readUntilSpace()
                self.timer.resume()
        
        isPause.setBlock()
        for p in self.players:
            p.close()
        self.timer.close()
        
    def initRun (self):
        
        # Build list of events
        for player in self.players:
            eventsInThis = player._getEvents ()
            if len(eventsInThis)==0:
                continue
            for evt in eventsInThis:
                e = {'timestamp': evt['timestamp'], 'id':evt['id'], 'object':player}
                self.eventList.append(e)
        self.eventList.sort(key=lambda e: e['timestamp'])
        if self.startTime == 0.0:
            for player in self.players:
                player.firstValidId = 0

        else:    
            validEvents = []
            start=self.eventList[0]['timestamp']
            for evt in self.eventList:
                if evt['timestamp'] < start + self.startTime:
                    continue
                else:
                    if evt['object'].firstValidId == -1:
                        evt['object'].firstValidId = evt['id']
                    validEvents.append(evt)
            
            self.eventList = validEvents

        # Tell data players to initialize
        for player in self.players:
            player.initializeRun()
        
        # Tell timer to initialize. Put a delay 1.s 
        self.timer = TimerProcess (
            [self.eventList[i]['timestamp'] for i in range(len(self.eventList))], 
            self.eventList[0]['timestamp']-1.0, 
            self.rate)
        
        
if __name__ == '__main__' :
    import sys
    import argparse
    
    argsp = argparse.ArgumentParser('Oxford ROS Player')
    argsp.add_argument('--dir', type=str, default=None, help='Directory of Oxford dataset')
    argsp.add_argument('--rate', type=float, default=1.0, help='Speed up/Slow down by rate factor')
    argsp.add_argument('--start', type=float, default=0.0, help='Start SEC seconds into dataset')
    args, unknown_args = argsp.parse_known_args()
    
    rospy.init_node('oxford_player', anonymous=True)
    player = PlayerControl (args.dir, rate=args.rate, start=args.start)
    poses = PosePlayer (player.dataset)
    images = ImagePlayer(player.dataset, raw=True)
    lidar3d = Lidar3Player (player.dataset)
    lidarfront = Lidar2Player (player.dataset)
    player.add_data_player(poses)
    player.add_data_player(images)
    player.add_data_player(lidar3d)
    player.add_data_player(lidarfront)
    
    print ("[SPACE] to pause, [Ctrl+C] to break")
    player.run()
    print ("Done")

