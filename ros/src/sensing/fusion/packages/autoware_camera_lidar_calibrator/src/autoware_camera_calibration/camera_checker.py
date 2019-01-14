#!/usr/bin/python
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

import cv2
import cv_bridge
import functools
import message_filters
import numpy
import rospy
import sensor_msgs.msg
import sensor_msgs.srv
import threading

from autoware_camera_calibration.calibrator import MonoCalibrator, StereoCalibrator, ChessboardInfo
from message_filters import ApproximateTimeSynchronizer

try:
    from queue import Queue
except ImportError:
    from Queue import Queue


def mean(seq):
    return sum(seq) / len(seq)

def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

class ConsumerThread(threading.Thread):
    def __init__(self, queue, function):
        threading.Thread.__init__(self)
        self.queue = queue
        self.function = function

    def run(self):
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                m = self.queue.get()
                if self.queue.empty():
                    break
            self.function(m)

class CameraCheckerNode:

    def __init__(self, chess_size, dim, approximate=0):
        self.board = ChessboardInfo()
        self.board.n_cols = chess_size[0]
        self.board.n_rows = chess_size[1]
        self.board.dim = dim

        image_topic = rospy.resolve_name("monocular") + "/image_rect"
        camera_topic = rospy.resolve_name("monocular") + "/camera_info"

        tosync_mono = [
            (image_topic, sensor_msgs.msg.Image),
            (camera_topic, sensor_msgs.msg.CameraInfo),
        ]

        if approximate <= 0:
            sync = message_filters.TimeSynchronizer
        else:
            sync = functools.partial(ApproximateTimeSynchronizer, slop=approximate)

        tsm = sync([message_filters.Subscriber(topic, type) for (topic, type) in tosync_mono], 10)
        tsm.registerCallback(self.queue_monocular)

        left_topic = rospy.resolve_name("stereo") + "/left/image_rect"
        left_camera_topic = rospy.resolve_name("stereo") + "/left/camera_info"
        right_topic = rospy.resolve_name("stereo") + "/right/image_rect"
        right_camera_topic = rospy.resolve_name("stereo") + "/right/camera_info"

        tosync_stereo = [
            (left_topic, sensor_msgs.msg.Image),
            (left_camera_topic, sensor_msgs.msg.CameraInfo),
            (right_topic, sensor_msgs.msg.Image),
            (right_camera_topic, sensor_msgs.msg.CameraInfo)
        ]

        tss = sync([message_filters.Subscriber(topic, type) for (topic, type) in tosync_stereo], 10)
        tss.registerCallback(self.queue_stereo)

        self.br = cv_bridge.CvBridge()

        self.q_mono = Queue()
        self.q_stereo = Queue()

        mth = ConsumerThread(self.q_mono, self.handle_monocular)
        mth.setDaemon(True)
        mth.start()

        sth = ConsumerThread(self.q_stereo, self.handle_stereo)
        sth.setDaemon(True)
        sth.start()

        self.mc = MonoCalibrator([self.board])
        self.sc = StereoCalibrator([self.board])

    def queue_monocular(self, msg, cmsg):
        self.q_mono.put((msg, cmsg))

    def queue_stereo(self, lmsg, lcmsg, rmsg, rcmsg):
        self.q_stereo.put((lmsg, lcmsg, rmsg, rcmsg))

    def mkgray(self, msg):
        return self.mc.mkgray(msg)

    def image_corners(self, im):
        (ok, corners, b) = self.mc.get_corners(im)
        if ok:
            return corners
        else:
            return None

    def handle_monocular(self, msg):

        (image, camera) = msg
        gray = self.mkgray(image)
        C = self.image_corners(gray)
        if C is not None:
            linearity_rms = self.mc.linear_error(C, self.board)

            # Add in reprojection check
            image_points = C
            object_points = self.mc.mk_object_points([self.board], use_board_size=True)[0]
            dist_coeffs = numpy.zeros((4, 1))
            camera_matrix = numpy.array( [ [ camera.P[0], camera.P[1], camera.P[2]  ],
                                           [ camera.P[4], camera.P[5], camera.P[6]  ],
                                           [ camera.P[8], camera.P[9], camera.P[10] ] ] )
            ok, rot, trans = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
            # Convert rotation into a 3x3 Rotation Matrix
            rot3x3, _ = cv2.Rodrigues(rot)
            # Reproject model points into image
            object_points_world = numpy.asmatrix(rot3x3) * numpy.asmatrix(object_points.squeeze().T) + numpy.asmatrix(trans)
            reprojected_h = camera_matrix * object_points_world
            reprojected   = (reprojected_h[0:2, :] / reprojected_h[2, :])
            reprojection_errors = image_points.squeeze().T - reprojected

            reprojection_rms = numpy.sqrt(numpy.sum(numpy.array(reprojection_errors) ** 2) / numpy.product(reprojection_errors.shape))

            # Print the results
            print("Linearity RMS Error: %.3f Pixels      Reprojection RMS Error: %.3f Pixels" % (linearity_rms, reprojection_rms))
        else:
            print('no chessboard')

    def handle_stereo(self, msg):

        (lmsg, lcmsg, rmsg, rcmsg) = msg
        lgray = self.mkgray(lmsg)
        rgray = self.mkgray(rmsg)

        L = self.image_corners(lgray)
        R = self.image_corners(rgray)
        if L is not None and R is not None:
            epipolar = self.sc.epipolar_error(L, R)

            dimension = self.sc.chessboard_size(L, R, self.board, msg=(lcmsg, rcmsg))

            print("epipolar error: %f pixels   dimension: %f m" % (epipolar, dimension))
        else:
            print("no chessboard")
