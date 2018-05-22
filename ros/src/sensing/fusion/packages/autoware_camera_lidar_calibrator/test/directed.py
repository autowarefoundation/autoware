#!/usr/bin/env python
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

import roslib
import rosunit
import rospy
import cv2

import collections
import copy
import numpy
import os
import sys
import tarfile
import unittest

from autoware_camera_calibration.calibrator import MonoCalibrator, StereoCalibrator, \
    Patterns, CalibrationException, ChessboardInfo, image_from_archive

board = ChessboardInfo()
board.n_cols = 8
board.n_rows = 6
board.dim = 0.108

class TestDirected(unittest.TestCase):
    def setUp(self):
        tar_path = roslib.packages.find_resource('autoware_camera_calibration', 'autoware_camera_calibration.tar.gz')[0]
        self.tar = tarfile.open(tar_path, 'r')
        self.limages = [image_from_archive(self.tar, "wide/left%04d.pgm" % i) for i in range(3, 15)]
        self.rimages = [image_from_archive(self.tar, "wide/right%04d.pgm" % i) for i in range(3, 15)]
        self.l = {}
        self.r = {}
        self.sizes = [(320,240), (640,480), (800,600), (1024,768)]
        for dim in self.sizes:
            self.l[dim] = []
            self.r[dim] = []
            for li,ri in zip(self.limages, self.rimages):
                rli = cv2.resize(li, (dim[0], dim[1]))
                rri = cv2.resize(ri, (dim[0], dim[1]))
                self.l[dim].append(rli)
                self.r[dim].append(rri)
                
    def assert_good_mono(self, c, dim, max_err):
        #c.report()
        self.assert_(len(c.ost()) > 0)
        lin_err = 0
        n = 0
        for img in self.l[dim]:
            lin_err_local = c.linear_error_from_image(img)
            if lin_err_local:
                lin_err += lin_err_local
                n += 1
        if n > 0:
            lin_err /= n
        self.assert_(0.0 < lin_err, 'lin_err is %f' % lin_err)
        self.assert_(lin_err < max_err, 'lin_err is %f' % lin_err)

        flat = c.remap(img)
        self.assertEqual(img.shape, flat.shape)

    def test_monocular(self):
        # Run the calibrator, produce a calibration, check it
        mc = MonoCalibrator([ board ], cv2.CALIB_FIX_K3)
        max_errs = [0.1, 0.2, 0.4, 0.7]
        for i, dim in enumerate(self.sizes):
            mc.cal(self.l[dim])
            self.assert_good_mono(mc, dim, max_errs[i])

            # Make another calibration, import previous calibration as a message,
            # and assert that the new one is good.

            mc2 = MonoCalibrator([board])
            mc2.from_message(mc.as_message())
            self.assert_good_mono(mc2, dim, max_errs[i])

    def test_stereo(self):
        epierrors = [0.1, 0.2, 0.45, 1.0]
        for i, dim in enumerate(self.sizes):
            print("Dim =", dim)
            sc = StereoCalibrator([board], cv2.CALIB_FIX_K3)
            sc.cal(self.l[dim], self.r[dim])

            sc.report()
            #print sc.ost()

            # NOTE: epipolar error currently increases with resolution.
            # At highest res expect error ~0.75
            epierror = 0
            n = 0
            for l_img, r_img in zip(self.l[dim], self.r[dim]):
                epierror_local = sc.epipolar_error_from_images(l_img, r_img)
                if epierror_local:
                    epierror += epierror_local
                    n += 1
            epierror /= n
            self.assert_(epierror < epierrors[i],
                         'Epipolar error is %f for resolution i = %d' % (epierror, i))

            self.assertAlmostEqual(sc.chessboard_size_from_images(self.l[dim][0], self.r[dim][0]), .108, 2)

            #print sc.as_message()

            img = self.l[dim][0]
            flat = sc.l.remap(img)
            self.assertEqual(img.shape, flat.shape)
            flat = sc.r.remap(img)
            self.assertEqual(img.shape, flat.shape)

            sc2 = StereoCalibrator([board])
            sc2.from_message(sc.as_message())
            # sc2.set_alpha(1.0)
            #sc2.report()
            self.assert_(len(sc2.ost()) > 0)

    def test_nochecker(self):
        # Run with same images, but looking for an incorrect chessboard size (8, 7).
        # Should raise an exception because of lack of input points.
        new_board = copy.deepcopy(board)
        new_board.n_cols = 8
        new_board.n_rows = 7

        sc = StereoCalibrator([new_board])
        self.assertRaises(CalibrationException, lambda: sc.cal(self.limages, self.rimages))
        mc = MonoCalibrator([new_board])
        self.assertRaises(CalibrationException, lambda: mc.cal(self.limages))



class TestArtificial(unittest.TestCase):
    Setup = collections.namedtuple('Setup', ['pattern', 'cols', 'rows', 'lin_err', 'K_err'])

    def setUp(self):
        # Define some image transforms that will simulate a camera position
        M = []
        cv2.getPerspectiveTransform
        self.K = numpy.array([[500, 0, 250], [0, 500, 250], [0, 0, 1]], numpy.float32)
        self.D = numpy.array([])
        # physical size of the board
        self.board_width_dim = 1

        # Generate data for different grid types. For each grid type, define the different sizes of
        # grid that are recognized (n row, n col)
        # Patterns.Circles, Patterns.ACircles
        self.setups = [ self.Setup(pattern=Patterns.Chessboard, cols=7, rows=8, lin_err=0.2, K_err=8.2),
                        self.Setup(pattern=Patterns.Circles, cols=7, rows=8, lin_err=0.1, K_err=4),
                        self.Setup(pattern=Patterns.ACircles, cols=3, rows=5, lin_err=0.1, K_err=8) ]
        self.limages = []
        self.rimages = []
        for setup in self.setups:
            self.limages.append([])
            self.rimages.append([])

            # Create the pattern
            if setup.pattern == Patterns.Chessboard:
                pattern = numpy.zeros((50*(setup.rows+3), 50*(setup.cols+3), 1), numpy.uint8)
                pattern.fill(255)
                for j in range(1, setup.rows+2):
                    for i in range(1+(j%2), setup.cols+2, 2):
                        pattern[50*j:50*(j+1), 50*i:50*(i+1)].fill(0)
            elif setup.pattern == Patterns.Circles:
                pattern = numpy.zeros((50*(setup.rows+2), 50*(setup.cols+2), 1), numpy.uint8)
                pattern.fill(255)
                for j in range(1, setup.rows+1):
                    for i in range(1, setup.cols+1):
                        cv2.circle(pattern, (50*i + 25, 50*j + 25), 15, (0,0,0), -1 )
            elif setup.pattern == Patterns.ACircles:
                x = 60
                pattern = numpy.zeros((x*(setup.rows+2), x*(setup.cols+5), 1), numpy.uint8)
                pattern.fill(255)
                for j in range(1, setup.rows+1):
                    for i in range(0, setup.cols):
                        cv2.circle(pattern, (x*(1 + 2*i + (j%2)) + x/2, x*j + x/2), x/3, (0,0,0), -1)

            rows, cols, _ = pattern.shape
            object_points_2d = numpy.array([[0, 0], [0, cols-1], [rows-1, cols-1], [rows-1, 0]], numpy.float32)
            object_points_3d = numpy.array([[0, 0, 0], [0, cols-1, 0], [rows-1, cols-1, 0], [rows-1, 0, 0]], numpy.float32)
            object_points_3d *= self.board_width_dim/float(cols)

            # create the artificial view points
            rvec = [ [0, 0, 0], [0, 0, 0.4], [0, 0.4, 0], [0.4, 0, 0], [0.4, 0.4, 0], [0.4, 0, 0.4], [0, 0.4, 0.4], [0.4, 0.4, 0.4] ]
            tvec = [ [-0.5, -0.5, 3], [-0.5, -0.5, 3], [-0.5, -0.1, 3], [-0.1, -0.5, 3], [-0.1, -0.1, 3], [-0.1, -0.5, 3], [-0.5, -0.1, 3], [-0.1, 0.1, 3] ]

            dsize = (480, 640)
            for i in range(len(rvec)):
                R = numpy.array(rvec[i], numpy.float32)
                T = numpy.array(tvec[i], numpy.float32)
            
                image_points, _ = cv2.projectPoints(object_points_3d, R, T, self.K, self.D)

                # deduce the perspective transform
                M.append(cv2.getPerspectiveTransform(object_points_2d, image_points))

                # project the pattern according to the different cameras
                pattern_warped = cv2.warpPerspective(pattern, M[i], dsize)
                self.limages[-1].append(pattern_warped)

    def assert_good_mono(self, c, images, max_err):
        #c.report()
        self.assert_(len(c.ost()) > 0)
        lin_err = 0
        n = 0
        for img in images:
            lin_err_local = c.linear_error_from_image(img)
            if lin_err_local:
                lin_err += lin_err_local
                n += 1
        if n > 0:
            lin_err /= n
        print("linear error is %f" % lin_err)
        self.assert_(0.0 < lin_err, 'lin_err is %f' % lin_err)
        self.assert_(lin_err < max_err, 'lin_err is %f' % lin_err)

        flat = c.remap(img)
        self.assertEqual(img.shape, flat.shape)

    def test_monocular(self):
        # Run the calibrator, produce a calibration, check it
        for i, setup in enumerate(self.setups):
            board = ChessboardInfo()
            board.n_cols = setup.cols
            board.n_rows = setup.rows
            board.dim = self.board_width_dim

            mc = MonoCalibrator([ board ], flags=cv2.CALIB_FIX_K3, pattern=setup.pattern)

            if 0:
                # display the patterns viewed by the camera
                for pattern_warped in self.limages[i]:
                    cv2.imshow("toto", pattern_warped)
                    cv2.waitKey(0)

            mc.cal(self.limages[i])
            self.assert_good_mono(mc, self.limages[i], setup.lin_err)

            # Make sure the intrinsics are similar
            err_intrinsics = numpy.linalg.norm(mc.intrinsics - self.K, ord=numpy.inf)
            self.assert_(err_intrinsics < setup.K_err,
                         'intrinsics error is %f for resolution i = %d' % (err_intrinsics, i))
            print('intrinsics error is %f' % numpy.linalg.norm(mc.intrinsics - self.K, ord=numpy.inf))

if __name__ == '__main__':
    #rosunit.unitrun('autoware_camera_calibration', 'directed', TestDirected)
    rosunit.unitrun('autoware_camera_calibration', 'artificial', TestArtificial)
