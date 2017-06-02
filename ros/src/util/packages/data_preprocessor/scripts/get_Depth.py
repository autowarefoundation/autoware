#!/usr/bin/env python
import sys
import rospy
import numpy as np
import time
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import cv2
import PIL.Image
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from scipy import ndimage

class DepthConverter(object):
    def __init__(self, calibration_path, save_path):
        self.calibration_path = calibration_path
        self.save_path = save_path
        self.CAMERA_EXTRINSIC_MAT = np.asmatrix(np.array(cv2.cv.Load(self.calibration_path, cv2.cv.CreateMemStorage(), "CameraExtrinsicMat")))
        self.CAMERA_MAT = np.array(cv2.cv.Load(self.calibration_path, cv2.cv.CreateMemStorage(), "CameraMat"))
        self.DIST_COEFF = np.transpose(np.array(cv2.cv.Load(self.calibration_path, cv2.cv.CreateMemStorage(), "DistCoeff")))
        inv_r = np.transpose(self.CAMERA_EXTRINSIC_MAT[0:3, 0:3])
        inv_t = -inv_r * (self.CAMERA_EXTRINSIC_MAT[0:3, 3])
        self.inv_r_T = np.transpose(inv_r)
        self.inv_t_T = np.transpose(inv_t)
        with open(calibration_path) as f:
            content = f.readlines()
        for line in content:
            if 'ImageSize' in line:
                width=int(line[line.index('[')+1:line.index(',')])
                height=int(line[line.index(',')+1:line.index(']')])
        print "Image Size: [",width,",",height,"]"
        self.width = width
        self.height = height
        self.index = 0
        self.in_callback = False
        self.cv_image = np.empty((self.height, self.width))
        self.cloud = PointCloud2()
        self.cloud_ready = False
        self.img_ready = False
        self.sub_mat_size = 40
        self.mat_ratio = 3
        self.max_color_val = 1

    def callback(self):
        self.in_callback = True
        curr_image = self.cv_image.copy()
        curr_cloud = self.cloud

        d_mat = np.empty((self.height,self.width))
        d_mat[:] = np.NaN

        xyz = pc2.read_points(curr_cloud, field_names=('x','y','z'))

        t0 = time.time()

        list_x=[]
        list_y=[]
        list_z=[]

        for p in xyz:
            if p[0]>0:
                list_x.append(p[0])
                list_y.append(p[1])
                list_z.append(p[2])

        c_length=len(list_x)
        points_mat=np.empty([c_length,3])

        for i, (x, y, z) in enumerate(zip(list_x,list_y,list_z)):
            points_mat[i,0]=x
            points_mat[i,1]=y
            points_mat[i,2]=z

        # projection from PointCloud to Depth
        for i in range(0,c_length):
            points_mat[i]=points_mat[i] * self.inv_r_T + self.inv_t_T

        tmp_x = points_mat[:,0:1] / points_mat[:,2:3]
        tmp_y = points_mat[:,1:2] / points_mat[:,2:3]

        r2 = tmp_x * tmp_x + tmp_y * tmp_y
        # caribration camera
        tmp_d= 1 + self.DIST_COEFF[0] * r2 + self.DIST_COEFF[1] * r2 * r2 + self.DIST_COEFF[4] * r2 * r2 * r2
        p_image_x = tmp_x * tmp_d + 2 * self.DIST_COEFF[2] * tmp_x * tmp_y + self.DIST_COEFF[3] * (r2 + 2 * tmp_x * tmp_x)
        p_image_y = tmp_y * tmp_d + self.DIST_COEFF[2] * (r2 + 2 * tmp_y * tmp_y) + 2 * self.DIST_COEFF[3] * tmp_x * tmp_y

        p_image_x = self.CAMERA_MAT[0, 0] * p_image_x + self.CAMERA_MAT[0, 2]
        p_image_y = self.CAMERA_MAT[1, 1] * p_image_y + self.CAMERA_MAT[1, 2]

        p_x = p_image_x + 0.5
        p_x=p_x.astype(int)
        p_y = p_image_y + 0.5
        p_y=p_y.astype(int)
        p_z = points_mat[:,2:3] # in cm

        for i_x, i_y, i_z in zip(p_x,p_y,p_z):
            if 0 <= i_x < self.width and 0 <= i_y < self.height:
                d_mat[i_y,i_x]=i_z

        t1 = time.time()
        print "Projection Time: ", t1-t0

        self.to_distance_image(d_mat,curr_image)
        self.in_callback = False
        self.reset_flags()

    def to_distance_image(self, img_mat,curr_image):
        self.index += 1
        h = self.height
        w = self.width
        print img_mat[np.isnan(img_mat)].shape, "nan"
        # print(np.nanmax(img_mat))
        # print(np.nanmin(img_mat))


        d_max=np.nanmax(img_mat)
        #d_max=d_max-3500  #optional. clips max distance so depth sense is more visible
        img_mat=img_mat/d_max

        new_mat=img_mat.copy()
        step_size = self.sub_mat_size/2

        t0 = time.time()

        # # Patty's interpolation implementation
        # for x in range(0,w,step_size):
        #    for y in range(0,h,step_size):
        #        sub_mat=img_mat[y:y+self.sub_mat_size*self.mat_ratio,x:x+self.sub_mat_size].copy()
        #        mask=np.isnan(sub_mat)
        #     #    if np.nanmin(sub_mat) < 0 and np.isnan(sub_mat).all() != np.nan:
        #     #        print(sub_mat[~mask])
        #     #        print("average")
        #     #        print(np.ma.average(np.ma.array(sub_mat,mask=mask)))
        #        avg=np.ma.average(np.ma.array(sub_mat,mask=mask))
        #        sub_mat.fill(avg)
        #        new_mat[y:y+self.sub_mat_size*self.mat_ratio,x:x+self.sub_mat_size]=sub_mat
        # t1 = time.time()
        # print "Interpolation time: ", t1-t0

        new_mat[np.isnan(new_mat)] = 0

        print new_mat.max()


        d_img = PIL.Image.fromarray(plt.cm.jet_r(new_mat, bytes=True))
        # rgba_img=PIL.Image.fromarray(curr_image).convert("RGBA")
        # overlay_img = PIL.Image.blend(rgba_img,d_img, 0.5)
        # print new_mat.max(), new_mat.min()
        # print plt.cm.jet_r(new_mat, bytes=True).max(), plt.cm.jet_r(new_mat, bytes=True).min()

        timestamp = time.time()
        d_img.save(self.save_path + '/depth_img_%08d.jpg' % self.index)
        # save_img = PIL.Image.fromarray(curr_image)
        # save_img.save(self.save_path + '/rgb_img_%08d.jpg' % self.index)
        name = self.save_path + '/rgb_img_%08d.jpg' % self.index
        cv2.imwrite(name, curr_image)

    def img_loader(self, image_msg):
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.img_ready = True

    def pc2_loader(self, msg):
        self.cloud = msg
        self.cloud_ready = True

    def reset_flags(self):
        self.in_callback = False
        self.img_ready = False
        self.cloud_ready = False

    def process(self, image, pointcloud):
        rospy.Subscriber(image, Image, self.img_loader)
        rospy.Subscriber(pointcloud, PointCloud2, self.pc2_loader)

        r=rospy.Rate(100) # TODO search best parameter  i think 50 is good
        while not rospy.is_shutdown():
            if self.cloud_ready and self.img_ready:
                self.callback()
            r.sleep()

def pc2_to_depth_image():
    rospy.init_node('pc2_to_depth_image', anonymous=True)

    try:
        save_path = sys.argv[1]
        calibration_path = sys.argv[2]
        image = sys.argv[3]
        pointcloud = sys.argv[4]
    except Exception, e:
        sys.exit("Please specify the save path. Example: rosbag_data_extract_unsync.py /media/0/output/")

    dc = DepthConverter(calibration_path, save_path)
    dc.process(image, pointcloud)

if __name__ == '__main__':
    pc2_to_depth_image()
