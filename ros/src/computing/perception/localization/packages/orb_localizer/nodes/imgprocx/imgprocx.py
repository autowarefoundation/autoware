#!/usr/bin/python

from __future__ import division
import cv2
from cv2 import cv
from copy import copy
import numpy as np
import multiprocessing.dummy as mp
from multiprocessing import Lock
import sys
import math

import rospy
import roslib
import cv_bridge
from sensor_msgs.msg import Image
from numpy import dtype


class ImageProcessor():
    
# Processing Mode
# mode = 0 : Do nothing
# mode = 1 : Automatic gamma adjustment
# mode = 2 : Convertion to Illumination-invariant color space

    def __init__ (self, processMode=0, subscribedImageTopic='', publishedImageTopic='', maskPath=None, **kwargs):
        self.imageSub = rospy.Subscriber(subscribedImageTopic, Image, self.imageCallback, queue_size=100)
        self.publisher = rospy.Publisher(publishedImageTopic, Image, queue_size=100)
        self.bridge = cv_bridge.CvBridge()
        self.mask = cv2.imread (maskPath, cv2.cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.cImage = None
        self.resultImage = None
        self.cImageHsv = None
        self.isMono = None
        self.imageMutex = Lock ()
        self.mode = processMode
        try:
            self.doSmearDetection = bool(kwargs['smearDetection'])
            self.iiAlpha = kwargs['IlluminatiAlpha']
        except KeyError:
            self.doSmearDetection = False
            self.iiAlpha = 0.394
#         cv2.namedWindow('xyz')
    
    def imageCallback (self, imageMsg):
#         print ("Callback called")
#         self.imageMutex.acquire()
        self.cImage = self.bridge.imgmsg_to_cv2(imageMsg, 'bgr8')
        self.isMono = False
#         self.imageMutex.release()
        
        self.process()
        
        if self.isMono:
            msg = self.bridge.cv2_to_imgmsg(self.resultImage, 'mono8')
        else:
            msg = self.bridge.cv2_to_imgmsg(self.resultImage, 'bgr8')
        msg.header.stamp = rospy.Time.now()
        self.publisher.publish(msg)

        
    def process (self):
        if self.cImage is None :
            return
        
#         self.imageMutex.acquire ()
        
        # Preparation
        imghsv = cv2.cvtColor(self.cImage, cv.CV_BGR2HSV)
        self.cImageHsv = cv2.split(imghsv)
        
        if self.doSmearDetection:
            smearQual = self.detectSmear(self.cImageHsv[2])
            print ("Smear: {}".format(smearQual))
            
        if self.mode == 0:
            self.resultImage = self.cImage
            self.isMono = False
            
        elif self.mode == 1 :
            self.resultImage = ImageProcessor.autoAdjustGammaRGB(self.cImage, self.mask)
            self.isMono = False
            
        elif self.mode == 2 :
            self.resultImage = ImageProcessor.toIlluminatiInvariant(self.cImage, self.iiAlpha)
            self.isMono = True
        
#         self.imageMutex.release ()
        

    @staticmethod
    def autoAdjustGammaRGB (rgbImage, mask=None, gammaOnly=False):
        monoimg = cv2.cvtColor(rgbImage, cv.CV_BGR2GRAY)
        g = ImageProcessor.autoAdjustGammaMono(monoimg, mask, gammaOnly=True)
        if gammaOnly:
            return g
        img_b = ImageProcessor.setGamma(rgbImage[:,:,0], g)
        img_g = ImageProcessor.setGamma(rgbImage[:,:,1], g)
        img_r = ImageProcessor.setGamma(rgbImage[:,:,2], g)
        return cv2.merge([img_b, img_g, img_r])
    

    @staticmethod
    def autoAdjustGammaMono (grayImage, mask=None, gammaOnly=False):
        roicdf = ImageProcessor.cdf(grayImage, mask)
#         Try to find midtone; it is X when cdf[X] = 0.5
        midtone = 0
        for i in range(len(roicdf)):
            if roicdf[i]>=0.5:
                midtone = i
                break
        target = 0.5
        midtone /= 255.0
        gamma = math.log(target) / math.log(midtone)
        if (gammaOnly):
            return gamma
        if (midtone >= 0.5):
            return grayImage
        return ImageProcessor.setGamma (grayImage, gamma)
        
    @staticmethod
    def toIlluminatiInvariant (imageRgb, alpha):
        imgf = np.array(imageRgb, dtype=np.float32) / 255.0
        imgf = 0.5 + np.log(imgf[:,:,1]) - alpha*np.log(imgf[:,:,0]) - (1-alpha)*np.log(imgf[:,:,2])
        return np.array(imgf*255.0, dtype=np.uint8)
#         return cv2.cvtColor(img, cv.CV_GRAY2BGR)



    def beautify (self):
        midall = self.allcdf[127]
        midroi = self.roicdf[127]
        
        def needBeautify ():
            if (midall >= 0.4 and midall <= 0.6):
                return False
            if (midall > 0.6 and midroi < midall):
                return False
            if (midall > 0.6 and midroi-midall>=0.16):
                return True
#         if abs(midall-midroi) < 
        
        if (needBeautify()):
            self.cImageHsv[2] = ImageProcessor.equalizeByMask(self.cImageHsv[2], self.mask)
            self.noProcess = False
        else:
            return

      
    @staticmethod
    def getNormalizedVerticalSum (chan):
        channorm = chan / 256.0
        tv = np.zeros(channorm.shape[1], dtype=np.float32)
        for i in range(channorm.shape[1]):
            tv[i] = np.sum(channorm[:,i])
        tv /= float(channorm.shape[0])
        return tv

    @staticmethod
    def detectSmear (VchanSrc, tolerance=0.1):
        threshold1 = 0.15 * VchanSrc.shape[1]
        # Normalize V channel
        tv = ImageProcessor.getNormalizedVerticalSum(VchanSrc)
        nc = 0
        for i in range(VchanSrc.shape[1]) :
            if tv[i] >= 1.0-tolerance:
                nc += 1
        if nc==0:
            return -1
        else:
            print ("Cols: {}".format(nc))
            if nc >= threshold1:
                return 1
            else:
                return float(nc) / float(threshold1) 
            
    @staticmethod
    def setGamma (source, gamma):
        LUT = np.array(
            [ ((i/255.0)**gamma)*255.0 for i in range(256)]
        , dtype=np.uint8)  
        return cv2.LUT(source, LUT)
        
    
    @staticmethod
    def equalizeByMask (source, mask=None):
        LUT = np.zeros((256,1), dtype=np.uint8)
        output = np.zeros(source.shape, dtype=source.dtype)
        hist = cv2.calcHist ([source], [0], mask, [256], [0,256])
        
        p = 0
        while (hist[p]==0):
            p += 1
        
        total = source.shape[0]*source.shape[1] 
        miz = hist[p]
        scale = 256.0 / (total-miz)
        sum = 0
        for i in range (p, 256):
            sum += int (hist[i])
            l = sum * scale
            if l > 255:
                LUT[i] = 255
            else:
                LUT[i] = l
                
        cv2.LUT(source, LUT, output)
        return output
#         for (LUT.at<uchar>(i++)=0; i<256; ++i) {
#             sum += (int)hist.at<float>(i);
#             LUT.at<uchar>(i) = cv::saturate_cast<uchar>(sum * scale);
#         }
    
#     def needEqualize (self):
#         pass
    
    @staticmethod
    def cdf (grayImage, mask=None, normalized=True):
        hist = cv2.calcHist ([grayImage], [0], mask, [256], [0,256])
        rcdf = np.cumsum(hist)
        if normalized:
            return rcdf / sum(hist)
        else:
            return rcdf


# class Downsampler:
#     def __init__ (self, imgproc, publishedTopic, rate=10.0):
#         self.rate = rospy.Rate(rate)
#         self.imgproc = imgproc
#         self.publisher = rospy.Publisher(publishedTopic, Image, queue_size=10)
#         self.bridge = cv_bridge.CvBridge()
#         
#         self.stop = False
#         self.process = mp.Process(target=self.start)
#         self.process.start()
#         
#     def start (self):
#         while self.stop==False:
#             
#             currentImage = None
#             self.imgproc.process()
#             currentImage = self.imgproc.resultImage
# 
#             if currentImage is not None:
#                 if self.imgproc.isMono:
#                     msg = self.bridge.cv2_to_imgmsg(currentImage, 'mono8')
#                 else:
#                     msg = self.bridge.cv2_to_imgmsg(currentImage, 'bgr8')
#                 msg.header.stamp = rospy.Time.now()
#                 self.publisher.publish(msg)
#             self.rate.sleep()


if __name__ == '__main__' :
    
    maskpath = sys.argv[1]
    
    mode = 0
    try:
        mode = int (sys.argv[2])
    except IndexError:
        pass
    if mode == 0:
        print ("Images will be untouched")
    else:
        print ("Images will be modified")
        
    rospy.init_node("imgprocx", anonymous=True)
    imgproc = ImageProcessor (mode, "/camera/image_raw", "/camera/image_hs", maskpath, smearDetection=False, IlluminatiAlpha=0.3975)
#     downsample = Downsampler (imgproc, "/camera/image_hs", rate=10.0)
    rospy.spin()
#     downsample.stop = True
    
    pass

