#!/usr/bin/env python
#
# Copyright 2018-2019 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#  v1.0 Jacob Lambert 2018-03-05
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import numpy

import cv2
import cv_bridge
import tarfile

from autoware_camera_calibration.calibrator import MonoCalibrator, StereoCalibrator, CalibrationException, ChessboardInfo

import rospy
import sensor_msgs.srv

def waitkey():
    k = cv2.waitKey(6)
    return k

def display(win_name, img):
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.imshow( win_name,  numpy.asarray( img[:,:] ))
    k=-1
    while k ==-1:
        k=waitkey()
    cv2.destroyWindow(win_name)
    if k in [27, ord('q')]:
        rospy.signal_shutdown('Quit')


def cal_from_tarfile(boards, tarname, mono = False, upload = False, calib_flags = 0, visualize = False, alpha=1.0):
    if mono:
        calibrator = MonoCalibrator(boards, calib_flags)
    else:
        calibrator = StereoCalibrator(boards, calib_flags)

    calibrator.do_tarfile_calibration(tarname)

    print(calibrator.ost())

    if upload: 
        info = calibrator.as_message()
        if mono:
            set_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("camera"), sensor_msgs.srv.SetCameraInfo)

            response = set_camera_info_service(info)
            if not response.success:
                raise RuntimeError("connected to set_camera_info service, but failed setting camera_info")
        else:
            set_left_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("left_camera"), sensor_msgs.srv.SetCameraInfo)
            set_right_camera_info_service = rospy.ServiceProxy("%s/set_camera_info" % rospy.remap_name("right_camera"), sensor_msgs.srv.SetCameraInfo)

            response1 = set_left_camera_info_service(info[0])
            response2 = set_right_camera_info_service(info[1])
            if not (response1.success and response2.success):
                raise RuntimeError("connected to set_camera_info service, but failed setting camera_info")

    if visualize:

        #Show rectified images
        calibrator.set_alpha(alpha)

        archive = tarfile.open(tarname, 'r')
        if mono:
            for f in archive.getnames():
                if f.startswith('left') and (f.endswith('.pgm') or f.endswith('png')):
                    filedata = archive.extractfile(f).read()
                    file_bytes = numpy.asarray(bytearray(filedata), dtype=numpy.uint8)
                    im=cv2.imdecode(file_bytes,cv2.IMREAD_COLOR)

                    bridge = cv_bridge.CvBridge()
                    try:
                        msg=bridge.cv2_to_imgmsg(im, "bgr8")
                    except cv_bridge.CvBridgeError as e:
                        print(e)

                    #handle msg returns the recitifed image with corner detection once camera is calibrated.
                    drawable=calibrator.handle_msg(msg)
                    vis=numpy.asarray( drawable.scrib[:,:])
                    #Display. Name of window:f
                    display(f, vis)
        else:
            limages = [ f for f in archive.getnames() if (f.startswith('left') and (f.endswith('pgm') or f.endswith('png'))) ]
            limages.sort()
            rimages = [ f for f in archive.getnames() if (f.startswith('right') and (f.endswith('pgm') or f.endswith('png'))) ]
            rimages.sort()

            if not len(limages) == len(rimages):
                raise RuntimeError("Left, right images don't match. %d left images, %d right" % (len(limages), len(rimages)))
            
            for i in range(len(limages)):
                l=limages[i]
                r=rimages[i]

                if l.startswith('left') and (l.endswith('.pgm') or l.endswith('png')) and r.startswith('right') and (r.endswith('.pgm') or r.endswith('png')):
                    # LEFT IMAGE
                    filedata = archive.extractfile(l).read()
                    file_bytes = numpy.asarray(bytearray(filedata), dtype=numpy.uint8)
                    im_left=cv2.imdecode(file_bytes,cv2.IMREAD_COLOR)
       
                    bridge = cv_bridge.CvBridge()
                    try:
                        msg_left=bridge.cv2_to_imgmsg(im_left, "bgr8")
                    except cv_bridge.CvBridgeError as e:
                        print(e)

                    #RIGHT IMAGE
                    filedata = archive.extractfile(r).read()
                    file_bytes = numpy.asarray(bytearray(filedata), dtype=numpy.uint8)
                    im_right=cv2.imdecode(file_bytes,cv2.IMREAD_COLOR)
                    try:
                        msg_right=bridge.cv2_to_imgmsg(im_right, "bgr8")
                    except cv_bridge.CvBridgeError as e:
                        print(e)

                    drawable=calibrator.handle_msg([ msg_left,msg_right] )

                    h, w = numpy.asarray(drawable.lscrib[:,:]).shape[:2]
                    vis = numpy.zeros((h, w*2, 3), numpy.uint8)
                    vis[:h, :w ,:] = numpy.asarray(drawable.lscrib[:,:])
                    vis[:h, w:w*2, :] = numpy.asarray(drawable.rscrib[:,:])
                    
                    display(l+" "+r,vis)    


if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser("%prog TARFILE [ opts ]")
    parser.add_option("--mono", default=False, action="store_true", dest="mono",
                      help="Monocular calibration only. Calibrates left images.")
    parser.add_option("-s", "--size", default=[], action="append", dest="size",
                      help="specify chessboard size as NxM [default: 8x6]")
    parser.add_option("-q", "--square", default=[], action="append", dest="square",
                      help="specify chessboard square size in meters [default: 0.108]")
    parser.add_option("--upload", default=False, action="store_true", dest="upload",
                      help="Upload results to camera(s).")
    parser.add_option("--fix-principal-point", action="store_true", default=False,
                     help="fix the principal point at the image center")
    parser.add_option("--fix-aspect-ratio", action="store_true", default=False,
                     help="enforce focal lengths (fx, fy) are equal")
    parser.add_option("--zero-tangent-dist", action="store_true", default=False,
                     help="set tangential distortion coefficients (p1, p2) to zero")
    parser.add_option("-k", "--k-coefficients", type="int", default=2, metavar="NUM_COEFFS",
                     help="number of radial distortion coefficients to use (up to 6, default %default)")
    parser.add_option("--visualize", action="store_true", default=False,
                     help="visualize rectified images after calibration")
    parser.add_option("-a", "--alpha", type="float", default=1.0, metavar="ALPHA",
                     help="zoom for visualization of rectifies images. Ranges from 0 (zoomed in, all pixels in calibrated image are valid) to 1 (zoomed out, all pixels in  original image are in calibrated image). default %default)")

    options, args = parser.parse_args()
    
    if len(options.size) != len(options.square):
        parser.error("Number of size and square inputs must be the same!")
    
    if not options.square:
        options.square.append("0.108")
        options.size.append("8x6")

    boards = []
    for (sz, sq) in zip(options.size, options.square):
        info = ChessboardInfo()
        info.dim = float(sq)
        size = tuple([int(c) for c in sz.split('x')])
        info.n_cols = size[0]
        info.n_rows = size[1]

        boards.append(info)

    if not boards:
        parser.error("Must supply at least one chessboard")

    if not args:
        parser.error("Must give tarfile name")
    if not os.path.exists(args[0]):
        parser.error("Tarfile %s does not exist. Please select valid tarfile" % args[0])

    tarname = args[0]

    num_ks = options.k_coefficients

    calib_flags = 0
    if options.fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if options.fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if options.zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if (num_ks > 3):
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if (num_ks < 6):
        calib_flags |= cv2.CALIB_FIX_K6
    if (num_ks < 5):
        calib_flags |= cv2.CALIB_FIX_K5
    if (num_ks < 4):
        calib_flags |= cv2.CALIB_FIX_K4
    if (num_ks < 3):
        calib_flags |= cv2.CALIB_FIX_K3
    if (num_ks < 2):
        calib_flags |= cv2.CALIB_FIX_K2
    if (num_ks < 1):
        calib_flags |= cv2.CALIB_FIX_K1

    cal_from_tarfile(boards, tarname, options.mono, options.upload, calib_flags, options.visualize, options.alpha)
