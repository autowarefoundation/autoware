#!/usr/bin/env python

import sys
import glob
import xml.etree.ElementTree as ET
import rospy
import tf
import numpy as np
import math
from std_msgs.msg import UInt32
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from jsk_recognition_msgs.msg import BoundingBox
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_rviz_plugins.msg import Pictogram
from jsk_rviz_plugins.msg import PictogramArray
#from calibration_camera_lidar.msg import projection_matrix
from cv_tracker.msg import image_obj
from cv_tracker.msg import image_rect

import os.path #Autoware
from numpy import dtype
import std_msgs

kitti_data = None
auto_boxes = None

pub = None
pub_boxes = None

pictogram_texts = None
pub_pictograms = None

rt_matrix = None #Autoware

velo_to_cam = None
cam_to_cam = None

# Wrap to [-pi..pi]
def wrapToPi(alpha):
	alpha = alpha % (2*math.pi)
	if alpha > math.pi:
		return alpha - 2*math.pi
	return alpha

def projectToImage(points_3d, K):
	points_2d = np.dot(K, points_3d[:3,:])
	points_2d[0,:]=points_2d[0,:]/points_2d[2,:]
	points_2d[1,:]=points_2d[1,:]/points_2d[2,:]
	return points_2d[:2,:]

def readXML(file):
	tree = ET.parse(file)
	root = tree.getroot()
	
	item = root.findall('./tracklets/item')

	d = {}
	boxes_2d = {}
	pictograms = {}

	for i, v in enumerate(item):
		h = float(v.find('h').text)
		w = float(v.find('w').text)
		l = float(v.find('l').text)
		frame = int(v.find('first_frame').text)
		size = Vector3(l, w, h)

		label = v.find('objectType').text

		pose = v.findall('./poses/item')

		for j, p in enumerate(pose):
			tx = float(p.find('tx').text)
			ty = float(p.find('ty').text)
			tz = float(p.find('tz').text)
			rz = float(p.find('rz').text)
			occlusion = float(p.find('occlusion').text)
			q = tf.transformations.quaternion_from_euler(0.0, 0.0, rz)

			b = BoundingBox()
			b.pose.position = Vector3(tx, ty, tz/2.0)
			b.pose.orientation = Quaternion(*q)
			b.dimensions = size
			b.label = i
			
			picto_text = Pictogram()
			picto_text.mode = Pictogram.STRING_MODE
			picto_text.pose.position = Vector3(tx, ty, -tz/2.0)
			q = tf.transformations.quaternion_from_euler(0.7, 0.0, -0.7)
			picto_text.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
			picto_text.size = 5
			picto_text.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
			picto_text.character = label
			
			# Bounding Box corners
			corner_x = np.array([l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2])
			corner_y = np.array([w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2])
			corner_z = np.array([0, 0, 0, 0, h, h, h, h])
			rz = wrapToPi(rz)
			
			###################
			#create box on origin, then translate and rotate according to pose. finally, project into 2D image
			# Rotate and translate 3D bounding box in velodyne coordinate system
			R = np.array([	[math.cos(rz), 	-math.sin(rz), 	0], 
							[math.sin(rz), 	math.cos(rz), 	0],
							[0, 			0, 				1]])
			corner_3d = np.dot(R,np.array([corner_x, corner_y, corner_z]))
			#Translate
			corner_3d[0,:] = corner_3d[0,:] + tx
			corner_3d[1,:] = corner_3d[1,:] + ty
			corner_3d[2,:] = corner_3d[2,:] + tz
			
			#Project to 2D
			low_row = np.vstack([corner_3d, np.ones(corner_3d.shape[1], dtype=np.float)])
			corner_3d = np.dot(np.asarray(rt_matrix), low_row)

			#################################
			#Create an orientation vector
			orientation_3d = np.dot( R, np.array([[0,0.7*l],[0,0],[0,0]]) )
			#Translate
			orientation_3d[0,:] = orientation_3d[0,:] + tx
			orientation_3d[1,:] = orientation_3d[1,:] + ty
			orientation_3d[2,:] = orientation_3d[2,:] + tz
			#Project
			low_row = np.vstack([orientation_3d, np.ones(orientation_3d.shape[1], dtype=np.float)])
			orientation_3d = np.dot(rt_matrix, low_row)

			K = np.asarray(cam_to_cam['P_rect_02']).reshape(3,4)
			K = K[:3,:3]

			corners_2d = projectToImage(corner_3d, K)
			orientation_2d = projectToImage(orientation_3d, K)

			x1 = min(corners_2d[0,:])
			x2 = max(corners_2d[0,:])
			y1 = min(corners_2d[1,:])
			y2 = max(corners_2d[1,:])

			bbox_2d = image_rect()
			bbox_2d.score = -10.0
			
			if ( (label == 'Car' or label=='Truck' or label=='Van') and np.any(corner_3d[2,:]>=0.5)) and (np.any(orientation_3d[2,:]>=0.5) and x1>=0 and x2>=0 and y1>0 and y2>=0 and occlusion <2):				
				bbox_2d.x = x1
				bbox_2d.y = y1
				bbox_2d.width = x2-x1
				bbox_2d.height = y2-y1
				bbox_2d.score = 1.0

			if d.has_key(frame + j) == True:
				d[frame + j].append(b)
				boxes_2d[frame + j].append(bbox_2d)
				pictograms[frame + j].append(picto_text)
			else:
				d[frame + j] = [b]
				boxes_2d[frame + j] = [bbox_2d]
				pictograms[frame + j]= [picto_text]

	return d, boxes_2d, pictograms

def callback(data):
	header = data.header
	frame = header.seq

	boxes = BoundingBoxArray() #3D Boxes with JSK
	boxes.header = header
	
	rects = image_obj() #Rects Autoware
	rects.header = header
	rects.type = "car"
	
	texts = PictogramArray() #Labels with JSK
	texts.header = header

	if kitti_data.has_key(frame) == True:
		for b in kitti_data[frame]:
			b.header = header
			boxes.boxes.append(b)

	if auto_boxes.has_key(frame) == True:
		for rect in auto_boxes[frame]:
			rects.obj.append(rect)
	
	if pictogram_texts.has_key(frame) == True:
		for txt in pictogram_texts[frame]:
			txt.header = header
			texts.pictograms.append(txt)

	pub.publish(boxes)
	pub_boxes.publish(rects)
	pub_pictograms.publish(texts)

def parseCalibrationFile(path):
	float_chars = set("0123456789.e+- ")
	data = {}
	with open(path, 'r') as f:
		for line in f.readlines():
			key, value = line.split(':', 1)
			value = value.strip()
			data[key] = value
			if float_chars.issuperset(value):
				# try to cast to float array
				try:
					data[key] = np.array(map(float, value.split(' ')))
				except ValueError:
					pass  # casting error: data[key] already eq. value, so pass

	return data

def publishProjectionMatrix(pathToCalibrationFile):
	global rt_matrix, velo_to_cam, cam_to_cam
	velo_to_cam = parseCalibrationFile(os.path.join(pathToCalibrationFile, "calib_velo_to_cam.txt"))
	cam_to_cam = parseCalibrationFile(os.path.join(pathToCalibrationFile, "calib_cam_to_cam.txt"))
	
	#projection_message = projection_matrix()
	rt_matrix = np.zeros((4,4))
	#rt_matrix[:3,:3] = np.transpose(velo_to_cam['R'].reshape(3,3)) #Rotation Matrix
	rt_matrix[:3,:3] = velo_to_cam['R'].reshape(3,3) #Rotation Matrix
	#rt_matrix[:3,3] = np.transpose(velo_to_cam['T']) #Translation in Augment Matrix
	rt_matrix[:3,3] = velo_to_cam['T'] #Translation in Augment Matrix
	rt_matrix[3,3] = 1 #Homogeneous coordinates
	
	#projection_message.projection_matrix = rt.reshape(-1,).tolist()
	#projection_publisher.publish(projection_message)

def run():
	global pub, pub_boxes, pub_pictograms, pub_points_clusters
	global projection_publisher
	rospy.init_node('kitti_box_publisher', anonymous=True)
	pub = rospy.Publisher('kitti_box', BoundingBoxArray, queue_size=1)
	pub_boxes = rospy.Publisher('/obj_car/image_obj', image_obj, queue_size=1)
	pub_pictograms = rospy.Publisher('kitti_3d_labels', PictogramArray, queue_size=1)
	pub_points_clusters = rospy.Publisher('points_cluster', PointCloud2, queue_size=1)
	#projection_publisher = rospy.Publisher('projection_matrix', projection_matrix, queue_size=1, latch=True)
	rospy.Subscriber("/kitti_player/hdl64e", PointCloud2, callback)
	
	
	rospy.spin()

if __name__ == "__main__":
	argv = sys.argv
	
	if len(argv) > 1:	
		xml_file = argv[1]
		publishProjectionMatrix(os.path.dirname(os.path.dirname(xml_file))) #Get parent directory
		kitti_data, auto_boxes, pictogram_texts = readXML(xml_file)
		run()

	else :
		print "[Usage] image.py tracklet_labels.xml"

