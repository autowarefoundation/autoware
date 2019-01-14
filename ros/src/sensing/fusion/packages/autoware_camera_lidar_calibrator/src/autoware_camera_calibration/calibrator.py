# !/usr/bin/env python
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

try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
from io import BytesIO
import cv2
import cv_bridge
import image_geometry
import math
import numpy.linalg
import pickle
import random
import sensor_msgs.msg
import tarfile
import time
import datetime
import StringIO as strio
from os import path
from distutils.version import LooseVersion


# Supported calibration patterns
class Patterns:
    Chessboard, Circles, ACircles = list(range(3))


class CalibrationException(Exception):
    pass


# TODO: Make pattern per-board?
class ChessboardInfo(object):
    def __init__(self, n_cols=0, n_rows=0, dim=0.0):
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.dim = dim


# Make all private!!!!!
def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]


def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]


def _pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))


def _get_outside_corners(corners, board):
    """
    Return the four corners of the board as a whole, as (up_left, up_right, down_right, down_left).
    """
    xdim = board.n_cols
    ydim = board.n_rows

    if corners.shape[1] * corners.shape[0] != xdim * ydim:
        raise Exception("Invalid number of corners! %d corners. X: %d, Y: %d" % (corners.shape[1] * corners.shape[0],
                                                                                 xdim, ydim))

    up_left = corners[0, 0]
    up_right = corners[xdim - 1, 0]
    down_right = corners[-1, 0]
    down_left = corners[-xdim, 0]

    return (up_left, up_right, down_right, down_left)


def _get_skew(corners, board):
    """
    Get skew for given checkerboard detection.
    Scaled to [0,1], which 0 = no skew, 1 = high skew
    Skew is proportional to the divergence of three outside corners from 90 degrees.
    """
    # TODO Using three nearby interior corners might be more robust, outside corners occasionally
    # get mis-detected
    up_left, up_right, down_right, _ = _get_outside_corners(corners, board)

    def angle(a, b, c):
        """
        Return angle between lines ab, bc
        """
        ab = a - b
        cb = c - b
        return math.acos(numpy.dot(ab, cb) / (numpy.linalg.norm(ab) * numpy.linalg.norm(cb)))

    skew = min(1.0, 2. * abs((math.pi / 2.) - angle(up_left, up_right, down_right)))
    return skew


def _get_area(corners, board):
    """
    Get 2d image area of the detected checkerboard.
    The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
    |p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.
    """
    (up_left, up_right, down_right, down_left) = _get_outside_corners(corners, board)
    a = up_right - up_left
    b = down_right - up_right
    c = down_left - down_right
    p = b + c
    q = a + b
    return abs(p[0] * q[1] - p[1] * q[0]) / 2.


def _get_corners(img, board, detection, refine=False, checkerboard_flags=0):
    """
    Get corners for a particular chessboard for an image
    """
    h = img.shape[0]
    w = img.shape[1]
    if len(img.shape) == 3 and img.shape[2] == 3:
        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        mono = img

    if detection == "cv2":
        (ok, corners) = cv2.findChessboardCorners(mono, (board.n_cols, board.n_rows),
                                                  flags=cv2.CALIB_CB_ADAPTIVE_THRESH |
                                                        cv2.CALIB_CB_NORMALIZE_IMAGE | checkerboard_flags)
    elif detection == "matlab":
        cv2.imwrite("temp_image.jpg", img)
        out = strio.StringIO()
        err = strio.StringIO()
        (mlcorners, boardsize, ok) = matlabeng.detectCheckerboardPoints("temp_image.jpg", nargout=3,stdout=out,stderr=err)
        ml_corners = numpy.array(mlcorners)
        if ml_corners.size != (2 * board.n_rows * board.n_cols):
            ok = 0
            return (ok, ml_corners)
        ml_corners = ml_corners.astype(numpy.float32).reshape((board.n_rows * board.n_cols, 1, 2))
        corners = ml_corners.copy()
        cv_idx = 0
        for i in range(0, board.n_rows):
            for j in range(0, board.n_cols):
                corners[cv_idx, :, :] = ml_corners[board.n_rows * j + i, :, :]
                cv_idx = cv_idx + 1

    if not ok:
        return (ok, corners)

    # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
    # NOTE: This may cause problems with very low-resolution cameras, where 8 pixels is a non-negligible fraction
    # of the image size. See http://answers.ros.org/question/3155/how-can-i-calibrate-low-resolution-cameras

    BORDER = 8
    if not all([(BORDER < corners[i, 0, 0] < (w - BORDER)) and (BORDER < corners[i, 0, 1] < (h - BORDER)) for i in
                range(corners.shape[0])]):
        ok = False

    # Ensure that all corner-arrays are going from top to bottom.
    if board.n_rows != board.n_cols:
        if corners[0, 0, 1] > corners[-1, 0, 1]:
            corners = numpy.copy(numpy.flipud(corners))
    else:
        direction_corners = (corners[-1] - corners[0]) >= numpy.array([[0.0, 0.0]])

        if not numpy.all(direction_corners):
            if not numpy.any(direction_corners):
                corners = numpy.copy(numpy.flipud(corners))
            elif direction_corners[0][0]:
                corners = numpy.rot90(corners.reshape(board.n_rows, board.n_cols, 2)).reshape(
                    board.n_cols * board.n_rows, 1, 2)
            else:
                corners = numpy.rot90(corners.reshape(board.n_rows, board.n_cols, 2), 3).reshape(
                    board.n_cols * board.n_rows, 1, 2)
    if refine and ok:
        # Use a radius of half the minimum distance between corners. This should be large enough to snap to the
        # correct corner, but not so large as to include a wrong corner in the search window.
        min_distance = float("inf")
        for row in range(board.n_rows):
            for col in range(board.n_cols - 1):
                index = row * board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index, 0], corners[index + 1, 0]))
        for row in range(board.n_rows - 1):
            for col in range(board.n_cols):
                index = row * board.n_rows + col
                min_distance = min(min_distance, _pdist(corners[index, 0], corners[index + board.n_cols, 0]))
        radius = int(math.ceil(min_distance * 0.5))
        cv2.cornerSubPix(mono, corners, (radius, radius), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))

    return (ok, corners)


def _get_circles(img, board, pattern):
    """
    Get circle centers for a symmetric or asymmetric grid
    """
    h = img.shape[0]
    w = img.shape[1]
    if len(img.shape) == 3 and img.shape[2] == 3:
        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        mono = img

    flag = cv2.CALIB_CB_SYMMETRIC_GRID
    if pattern == Patterns.ACircles:
        flag = cv2.CALIB_CB_ASYMMETRIC_GRID
    mono_arr = numpy.array(mono)
    (ok, corners) = cv2.findCirclesGrid(mono_arr, (board.n_cols, board.n_rows), flags=flag)

    # In symmetric case, findCirclesGrid does not detect the target if it's turned sideways. So we try
    # again with dimensions swapped - not so efficient.
    # TODO Better to add as second board? Corner ordering will change.
    if not ok and pattern == Patterns.Circles:
        (ok, corners) = cv2.findCirclesGrid(mono_arr, (board.n_rows, board.n_cols), flags=flag)

    return (ok, corners)


# TODO self.size needs to come from CameraInfo, full resolution
class Calibrator(object):
    """
    Base class for calibration system
    """

    def __init__(self, boards, flags=0, pattern=Patterns.Chessboard, name='', detection='cv2', output='yaml',
                 checkerboard_flags=cv2.CALIB_CB_FAST_CHECK, min_good_enough=40):
        # Ordering the dimensions for the different detectors is actually a minefield...
        if pattern == Patterns.Chessboard:
            # Make sure n_cols > n_rows to agree with OpenCV CB detector output
            self._boards = [ChessboardInfo(max(i.n_cols, i.n_rows), min(i.n_cols, i.n_rows), i.dim) for i in boards]
        elif pattern == Patterns.ACircles:
            # 7x4 and 4x7 are actually different patterns. Assume square-ish pattern, so n_rows > n_cols.
            self._boards = [ChessboardInfo(min(i.n_cols, i.n_rows), max(i.n_cols, i.n_rows), i.dim) for i in boards]
        elif pattern == Patterns.Circles:
            # We end up having to check both ways anyway
            self._boards = boards

        # Set to true after we perform calibration
        self.calibrated = False
        self.calib_flags = flags
        self.checkerboard_flags = checkerboard_flags
        self.pattern = pattern
        self.br = cv_bridge.CvBridge()
        self.min_good_enough = min_good_enough

        # self.db is list of (parameters, image) samples for use in calibration. parameters has form
        # (X, Y, size, skew) all normalized to [0,1], to keep track of what sort of samples we've taken
        # and ensure enough variety.
        self.db = []
        # For each db sample, we also record the detected corners.
        self.good_corners = []
        # Set to true when we have sufficiently varied samples to calibrate
        self.goodenough = False
        self.param_ranges = [0.7, 0.7, 0.4, 0.5]
        self.name = name
        self.detection = detection
        if detection == "matlab":
            import matlab.engine
            global matlabeng
            matlabeng = matlab.engine.start_matlab()  # because start matlab is super slow
        self.output = output

    def mkgray(self, msg):
        """
        Convert a message into a 8-bit 1 channel monochrome OpenCV image
        """
        # as cv_bridge automatically scales, we need to remove that behavior
        # TODO: get a Python API in cv_bridge to check for the image depth.
        if self.br.encoding_to_dtype_with_channels(msg.encoding)[0] in ['uint16', 'int16']:
            mono16 = self.br.imgmsg_to_cv2(msg, '16UC1')
            mono8 = numpy.array(numpy.clip(mono16, 0, 255), dtype=numpy.uint8)
            return mono8
        elif 'FC1' in msg.encoding:
            # floating point image handling
            img = self.br.imgmsg_to_cv2(msg, "passthrough")
            _, max_val, _, _ = cv2.minMaxLoc(img)
            if max_val > 0:
                scale = 255.0 / max_val
                mono_img = (img * scale).astype(numpy.uint8)
            else:
                mono_img = img.astype(numpy.uint8)
            return mono_img
        else:
            return self.br.imgmsg_to_cv2(msg, "mono8")

    def get_parameters(self, corners, board, size):
        """
        Return list of parameters [X, Y, size, skew] describing the checkerboard view.
        """
        (width, height) = size
        Xs = corners[:, :, 0]
        Ys = corners[:, :, 1]
        area = _get_area(corners, board)
        border = math.sqrt(area)
        # For X and Y, we "shrink" the image all around by approx. half the board size.
        # Otherwise large boards are penalized because you can't get much X/Y variation.
        p_x = min(1.0, max(0.0, (numpy.mean(Xs) - border / 2) / (width - border)))
        p_y = min(1.0, max(0.0, (numpy.mean(Ys) - border / 2) / (height - border)))
        p_size = math.sqrt(area / (width * height))
        skew = _get_skew(corners, board)
        params = [p_x, p_y, p_size, skew]
        return params

    def is_good_sample(self, params):
        """
        Returns true if the checkerboard detection described by params should be added to the database.
        """
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a - b) for (a, b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        # print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        return d > 0.2

    _param_names = ["X", "Y", "Size", "Skew"]

    def compute_goodenough(self):
        if not self.db:
            return None

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = all_params[0]
        max_params = all_params[0]
        for params in all_params[1:]:
            min_params = lmin(min_params, params)
            max_params = lmax(max_params, params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.param_ranges)]
        # If we have lots of samples, allow calibration even if not all parameters are green
        # TODO Awkward that we update self.goodenough instead of returning it
        self.goodenough = (len(self.db) >= self.min_good_enough) or all([p == 1.0 for p in progress])

        return list(zip(self._param_names, min_params, max_params, progress))

    def mk_object_points(self, boards, use_board_size=False):
        opts = []
        for i, b in enumerate(boards):
            num_pts = b.n_cols * b.n_rows
            opts_loc = numpy.zeros((num_pts, 1, 3), numpy.float32)
            for j in range(num_pts):
                opts_loc[j, 0, 0] = (j / b.n_cols)
                if self.pattern == Patterns.ACircles:
                    opts_loc[j, 0, 1] = 2 * (j % b.n_cols) + (opts_loc[j, 0, 0] % 2)
                else:
                    opts_loc[j, 0, 1] = (j % b.n_cols)
                opts_loc[j, 0, 2] = 0
                if use_board_size:
                    opts_loc[j, 0, :] = opts_loc[j, 0, :] * b.dim
            opts.append(opts_loc)
        return opts

    def get_corners(self, img, refine=True):
        """
        Use cvFindChessboardCorners to find corners of chessboard in image.

        Check all boards. Return corners for first chessboard that it detects
        if given multiple size chessboards.

        Returns (ok, corners, board)
        """

        for b in self._boards:
            if self.pattern == Patterns.Chessboard:
                (ok, corners) = _get_corners(img, b, self.detection, True, self.checkerboard_flags)
            else:
                (ok, corners) = _get_circles(img, b, self.pattern)
            if ok:
                return (ok, corners, b)
        return (False, None, None)

    def downsample_and_detect(self, img):
        """
        Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Combines these apparently orthogonal duties as an optimization. Checkerboard
        detection is too expensive on large images, so it's better to do detection on
        the smaller display image and scale the corners back up to the correct size.

        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale)).
        """
        # Scale the input image down to ~VGA size
        height = img.shape[0]
        width = img.shape[1]
        scale = math.sqrt((width * height) / (640. * 480.))
        if scale > 1.0:
            scrib = cv2.resize(img, (int(width / scale), int(height / scale)))
        else:
            scrib = img
        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(width) / scrib.shape[1]
        y_scale = float(height) / scrib.shape[0]

        if self.pattern == Patterns.Chessboard:
            # Detect checkerboard
            (ok, downsampled_corners, board) = self.get_corners(scrib, refine=True)

            # Scale corners back to full size image
            corners = None
            if ok:
                if scale > 1.0:
                    # Refine up-scaled corners in the original full-res image
                    # TODO Does this really make a difference in practice?
                    corners_unrefined = downsampled_corners.copy()
                    corners_unrefined[:, :, 0] *= x_scale
                    corners_unrefined[:, :, 1] *= y_scale
                    radius = int(math.ceil(scale))
                    if len(img.shape) == 3 and img.shape[2] == 3:
                        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    else:
                        mono = img
                    cv2.cornerSubPix(mono, corners_unrefined, (radius, radius), (-1, -1),
                                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
                    corners = corners_unrefined
                else:
                    corners = downsampled_corners
        else:
            # Circle grid detection is fast even on large images
            (ok, corners, board) = self.get_corners(img)
            # Scale corners to downsampled image for display
            downsampled_corners = None
            if ok:
                if scale > 1.0:
                    downsampled_corners = corners.copy()
                    downsampled_corners[:, :, 0] /= x_scale
                    downsampled_corners[:, :, 1] /= y_scale
                else:
                    downsampled_corners = corners

        return (scrib, corners, downsampled_corners, board, (x_scale, y_scale))

    def lrmsg(self, d, k, r, p):
        """ Used by :meth:`as_message`.  Return a CameraInfo message for the given calibration matrices """
        msg = sensor_msgs.msg.CameraInfo()
        (msg.width, msg.height) = self.size
        if d.size > 5:
            msg.distortion_model = "rational_polynomial"
        else:
            msg.distortion_model = "plumb_bob"
        msg.D = numpy.ravel(d).copy().tolist()
        msg.K = numpy.ravel(k).copy().tolist()
        msg.R = numpy.ravel(r).copy().tolist()
        msg.P = numpy.ravel(p).copy().tolist()
        return msg

    def lrreport(self, d, k, r, p):
        print("D = ", numpy.ravel(d).tolist())
        print("K = ", numpy.ravel(k).tolist())
        print("R = ", numpy.ravel(r).tolist())
        print("P = ", numpy.ravel(p).tolist())

    def lrost(self, name, d, k, r, p):
        calmessage = (
            "# oST version 5.0 parameters\n"
            + "\n"
            + "\n"
            + "[image]\n"
            + "\n"
            + "width\n"
            + str(self.size[0]) + "\n"
            + "\n"
            + "height\n"
            + str(self.size[1]) + "\n"
            + "\n"
            + "[%s]" % name + "\n"
            + "\n"
            + "camera matrix\n"
            + " ".join(["%8f" % k[0, i] for i in range(3)]) + "\n"
            + " ".join(["%8f" % k[1, i] for i in range(3)]) + "\n"
            + " ".join(["%8f" % k[2, i] for i in range(3)]) + "\n"
            + "\n"
            + "distortion\n"
            + " ".join(["%8f" % d[i, 0] for i in range(d.shape[0])]) + "\n"
            + "\n"
            + "rectification\n"
            + " ".join(["%8f" % r[0, i] for i in range(3)]) + "\n"
            + " ".join(["%8f" % r[1, i] for i in range(3)]) + "\n"
            + " ".join(["%8f" % r[2, i] for i in range(3)]) + "\n"
            + "\n"
            + "projection\n"
            + " ".join(["%8f" % p[0, i] for i in range(4)]) + "\n"
            + " ".join(["%8f" % p[1, i] for i in range(4)]) + "\n"
            + " ".join(["%8f" % p[2, i] for i in range(4)]) + "\n"
            + "\n")
        assert len(calmessage) < 525, "Calibration info must be less than 525 bytes"
        return calmessage

    def lryaml(self, name, d, k, r, p):
        calmessage = (""
                      + "image_width: " + str(self.size[0]) + "\n"
                      + "image_height: " + str(self.size[1]) + "\n"
                      + "camera_name: " + name + "\n"
                      + "camera_matrix:\n"
                      + "  rows: 3\n"
                      + "  cols: 3\n"
                      + "  data: [" + ", ".join(["%8f" % i for i in k.reshape(1, 9)[0]]) + "]\n"
                      + "distortion_model: " + ("rational_polynomial" if d.size > 5 else "plumb_bob") + "\n"
                      + "distortion_coefficients:\n"
                      + "  rows: 1\n"
                      + "  cols: 5\n"
                      + "  data: [" + ", ".join(["%8f" % d[i, 0] for i in range(d.shape[0])]) + "]\n"
                      + "rectification_matrix:\n"
                      + "  rows: 3\n"
                      + "  cols: 3\n"
                      + "  data: [" + ", ".join(["%8f" % i for i in r.reshape(1, 9)[0]]) + "]\n"
                      + "projection_matrix:\n"
                      + "  rows: 3\n"
                      + "  cols: 4\n"
                      + "  data: [" + ", ".join(["%8f" % i for i in p.reshape(1, 12)[0]]) + "]\n"
                      + "")
        return calmessage

    def do_save(self):
        now = datetime.datetime.now()
        filename = path.join(path.expanduser("~"), now.strftime("%Y%m%d_%H%M_") + 'calibrationdata.tar.gz')
        tf = tarfile.open(filename, 'w:gz')
        self.do_tarfile_save(tf)  # Must be overridden in subclasses
        tf.close()
        print("Wrote calibration data to: " + filename)


def image_from_archive(archive, name):
    """
    Load image PGM file from tar archive.

    Used for tarfile loading and unit test.
    """
    member = archive.getmember(name)
    imagefiledata = numpy.fromstring(archive.extractfile(member).read(), numpy.uint8)
    imagefiledata.resize((1, imagefiledata.size))
    return cv2.imdecode(imagefiledata, cv2.IMREAD_COLOR)


class ImageDrawable(object):
    """
    Passed to CalibrationNode after image handled. Allows plotting of images
    with detected corner points
    """

    def __init__(self):
        self.params = None


class MonoDrawable(ImageDrawable):
    def __init__(self):
        ImageDrawable.__init__(self)
        self.scrib = None
        self.linear_error = -1.0


class StereoDrawable(ImageDrawable):
    def __init__(self):
        ImageDrawable.__init__(self)
        self.lscrib = None
        self.rscrib = None
        self.epierror = -1
        self.dim = -1


class MonoCalibrator(Calibrator):
    """
    Calibration class for monocular cameras::

        images = [cv2.imread("mono%d.png") for i in range(8)]
        mc = MonoCalibrator()
        mc.cal(images)
        print mc.as_message()
    """

    is_mono = True  # TODO Could get rid of is_mono

    def __init__(self, *args, **kwargs):
        if 'name' not in kwargs:
            kwargs['name'] = 'camera_mono'
        super(MonoCalibrator, self).__init__(*args, **kwargs)

    def cal(self, images):
        """
        Calibrate camera from given images
        """
        goodcorners = self.collect_corners(images)
        self.cal_fromcorners(goodcorners)
        self.calibrated = True

    def collect_corners(self, images):
        """
        :param images: source images containing chessboards
        :type images: list of :class:`cvMat`

        Find chessboards in all images.

        Return [ (corners, ChessboardInfo) ]
        """
        self.size = (images[0].shape[1], images[0].shape[0])
        corners = [self.get_corners(i) for i in images]

        goodcorners = [(co, b) for (ok, co, b) in corners if ok]
        if not goodcorners:
            raise CalibrationException("No corners found in images!")
        return goodcorners

    def cal_fromcorners(self, good):
        """
        :param good: Good corner positions and boards
        :type good: [(corners, ChessboardInfo)]


        """
        boards = [b for (_, b) in good]

        ipts = [points for (points, _) in good]
        opts = self.mk_object_points(boards)

        self.intrinsics = numpy.zeros((3, 3), numpy.float64)
        if self.calib_flags & cv2.CALIB_RATIONAL_MODEL:
            self.distortion = numpy.zeros((8, 1), numpy.float64)  # rational polynomial
        else:
            self.distortion = numpy.zeros((5, 1), numpy.float64)  # plumb bob
        # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
        self.intrinsics[0, 0] = 1.0
        self.intrinsics[1, 1] = 1.0
        cv2.calibrateCamera(
            opts, ipts,
            self.size, self.intrinsics,
            self.distortion,
            flags=self.calib_flags)

        # R is identity matrix for monocular calibration
        self.R = numpy.eye(3, dtype=numpy.float64)
        self.P = numpy.zeros((3, 4), dtype=numpy.float64)

        self.set_alpha(0.0)

    def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution.  The alpha
        value is a zoom, and ranges from 0 (zoomed in, all pixels in
        calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """

        # NOTE: Prior to Electric, this code was broken such that we never actually saved the new
        # camera matrix. In effect, this enforced P = [K|0] for monocular cameras.
        # TODO: Verify that OpenCV #1199 gets applied (improved GetOptimalNewCameraMatrix)
        ncm, _ = cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, a)
        for j in range(3):
            for i in range(3):
                self.P[j, i] = ncm[j, i]
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.intrinsics, self.distortion, self.R, ncm, self.size,
                                                           cv2.CV_32FC1)

    def remap(self, src):
        """
        :param src: source image
        :type src: :class:`cvMat`

        Apply the post-calibration undistortion to the source image
        """
        return cv2.remap(src, self.mapx, self.mapy, cv2.INTER_LINEAR)

    def undistort_points(self, src):
        """
        :param src: N source pixel points (u,v) as an Nx2 matrix
        :type src: :class:`cvMat`

        Apply the post-calibration undistortion to the source points
        """

        return cv2.undistortPoints(src, self.intrinsics, self.distortion, R=self.R, P=self.P)

    def as_message(self):
        """ Return the camera calibration as a CameraInfo message """
        return self.lrmsg(self.distortion, self.intrinsics, self.R, self.P)

    def from_message(self, msg, alpha=0.0):
        """ Initialize the camera calibration from a CameraInfo message """

        self.size = (msg.width, msg.height)
        self.intrinsics = numpy.array(msg.K, dtype=numpy.float64, copy=True).reshape((3, 3))
        self.distortion = numpy.array(msg.D, dtype=numpy.float64, copy=True).reshape((len(msg.D), 1))
        self.R = numpy.array(msg.R, dtype=numpy.float64, copy=True).reshape((3, 3))
        self.P = numpy.array(msg.P, dtype=numpy.float64, copy=True).reshape((3, 4))

        self.set_alpha(0.0)

    def report(self):
        self.lrreport(self.distortion, self.intrinsics, self.R, self.P)

    def ost(self):
        return self.lrost(self.name, self.distortion, self.intrinsics, self.R, self.P)

    def yaml(self):
        return self.lryaml(self.name, self.distortion, self.intrinsics, self.R, self.P)

    def linear_error_from_image(self, image):
        """
        Detect the checkerboard and compute the linear error.
        Mainly for use in tests.
        """
        _, corners, _, board, _ = self.downsample_and_detect(image)
        if corners is None:
            return None

        undistorted = self.undistort_points(corners)
        return self.linear_error(undistorted, board)

    @staticmethod
    def linear_error(corners, b):

        """
        Returns the linear error for a set of corners detected in the unrectified image.
        """

        if corners is None:
            return None

        def pt2line(x0, y0, x1, y1, x2, y2):
            """ point is (x0, y0), line is (x1, y1, x2, y2) """
            return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        cc = b.n_cols
        cr = b.n_rows
        errors = []
        for r in range(cr):
            (x1, y1) = corners[(cc * r) + 0, 0]
            (x2, y2) = corners[(cc * r) + cc - 1, 0]
            for i in range(1, cc - 1):
                (x0, y0) = corners[(cc * r) + i, 0]
                errors.append(pt2line(x0, y0, x1, y1, x2, y2))
        if errors:
            return math.sqrt(sum([e ** 2 for e in errors]) / len(errors))
        else:
            return None

    def handle_msg(self, msg):
        """
        Detects the calibration target and, if found and provides enough new information,
        adds it to the sample database.

        Returns a MonoDrawable message with the display image and progress info.
        """
        gray = self.mkgray(msg)
        linear_error = -1

        # Get display-image-to-be (scrib) and detection of the calibration target
        scrib_mono, corners, downsampled_corners, board, (x_scale, y_scale) = self.downsample_and_detect(gray)

        if self.calibrated:
            # Show rectified image
            # TODO Pull out downsampling code into function
            gray_remap = self.remap(gray)
            gray_rect = gray_remap
            if x_scale != 1.0 or y_scale != 1.0:
                gray_rect = cv2.resize(gray_remap, (scrib_mono.shape[1], scrib_mono.shape[0]))

            scrib = cv2.cvtColor(gray_rect, cv2.COLOR_GRAY2BGR)

            if corners is not None:
                # Report linear error
                undistorted = self.undistort_points(corners)
                linear_error = self.linear_error(undistorted, board)

                # Draw rectified corners
                scrib_src = undistorted.copy()
                scrib_src[:, :, 0] /= x_scale
                scrib_src[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(scrib, (board.n_cols, board.n_rows), scrib_src, True)

        else:
            scrib = cv2.cvtColor(scrib_mono, cv2.COLOR_GRAY2BGR)
            if corners is not None:
                # Draw (potentially downsampled) corners onto display image
                cv2.drawChessboardCorners(scrib, (board.n_cols, board.n_rows), downsampled_corners, True)

                # Add sample to database only if it's sufficiently different from any previous sample.
                params = self.get_parameters(corners, board, (gray.shape[1], gray.shape[0]))
                if self.is_good_sample(params):
                    self.db.append((params, gray))
                    self.good_corners.append((corners, board))
                    print(("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple(
                        [len(self.db)] + params)))

        rv = MonoDrawable()
        rv.scrib = scrib
        rv.params = self.compute_goodenough()
        rv.linear_error = linear_error
        return rv

    def do_calibration(self, dump=False):
        if not self.good_corners:
            print("**** Collecting corners for all images! ****")  # DEBUG
            images = [i for (p, i) in self.db]
            self.good_corners = self.collect_corners(images)
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, self.good_corners),
                        open("camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.size = (self.db[0][1].shape[1], self.db[0][1].shape[0])  # TODO Needs to be set externally
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        print((self.report()))
        print((self.ost()))

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """

        def taradd(name, buf):
            if isinstance(buf, basestring):
                s = StringIO(buf)
            else:
                s = BytesIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.getvalue())
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        ims = [("left-%04d.png" % i, im) for i, (_, im) in enumerate(self.db)]
        for (name, im) in ims:
            taradd(name, cv2.imencode(".png", im)[1].tostring())
        if self.output == 'yaml':
            self.do_autoware_cv_yaml()
        else:
            taradd('ost.yaml', self.yaml())
            taradd('ost.txt', self.ost())

    def do_autoware_cv_yaml(self):
        # params to be written
        camera_name = self.name
        distortion = self.distortion
        intrinsics = self.intrinsics
        reproj_error = 0.0      # until we have an alternative..

        # write file
        now = datetime.datetime.now()
        fn = path.join(path.expanduser("~"), now.strftime("%Y%m%d_%H%M_") + camera_name + '.yaml')
        f = cv2.FileStorage(fn, flags=1)
        f.write(name="CameraExtrinsicMat", val=numpy.eye(4))
        f.write(name="CameraMat", val=intrinsics)
        f.write(name="DistCoeff", val=distortion)
        f.release()
        with open(fn, 'a') as f:
            f.write("ImageSize: [" + str(self.size[0]) + ", " + str(self.size[1]) + "]\n")
            f.write("Reprojection Error: " + str(reproj_error) + "\n")
            f.write("DistModel: plumb_bob")
        print("Wrote Autoware Calibration file in: " + fn)

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, 'r')

        limages = [image_from_archive(archive, f) for f in archive.getnames() if
                   (f.startswith('left') and (f.endswith('.pgm') or f.endswith('png')))]

        self.cal(limages)


# TODO Replicate MonoCalibrator improvements in stereo
class StereoCalibrator(Calibrator):
    """
    Calibration class for stereo cameras::

        limages = [cv2.imread("left%d.png") for i in range(8)]
        rimages = [cv2.imread("right%d.png") for i in range(8)]
        sc = StereoCalibrator()
        sc.cal(limages, rimages)
        print sc.as_message()
    """

    is_mono = False

    def __init__(self, *args, **kwargs):
        if 'name' not in kwargs:
            kwargs['name'] = 'narrow_stereo'
        super(StereoCalibrator, self).__init__(*args, **kwargs)
        self.l = MonoCalibrator(*args, **kwargs)
        self.r = MonoCalibrator(*args, **kwargs)
        # Collecting from two cameras in a horizontal stereo rig, can't get
        # full X range in the left camera.
        self.param_ranges[0] = 0.4

    def cal(self, limages, rimages):
        """
        :param limages: source left images containing chessboards
        :type limages: list of :class:`cvMat`
        :param rimages: source right images containing chessboards
        :type rimages: list of :class:`cvMat`

        Find chessboards in images, and runs the OpenCV calibration solver.
        """
        goodcorners = self.collect_corners(limages, rimages)
        self.size = (limages[0].shape[1], limages[0].shape[0])
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(goodcorners)
        self.calibrated = True

    def collect_corners(self, limages, rimages):
        """
        For a sequence of left and right images, find pairs of images where both
        left and right have a chessboard, and return  their corners as a list of pairs.
        """
        # Pick out (corners, board) tuples
        lcorners = [self.downsample_and_detect(i)[1:4:2] for i in limages]
        rcorners = [self.downsample_and_detect(i)[1:4:2] for i in rimages]
        good = [(lco, rco, b) for ((lco, b), (rco, br)) in zip(lcorners, rcorners)
                if (lco is not None and rco is not None)]

        if len(good) == 0:
            raise CalibrationException("No corners found in images!")
        return good

    def cal_fromcorners(self, good):
        # Perform monocular calibrations
        lcorners = [(l, b) for (l, r, b) in good]
        rcorners = [(r, b) for (l, r, b) in good]
        self.l.cal_fromcorners(lcorners)
        self.r.cal_fromcorners(rcorners)

        lipts = [l for (l, _, _) in good]
        ripts = [r for (_, r, _) in good]
        boards = [b for (_, _, b) in good]

        opts = self.mk_object_points(boards, True)

        flags = cv2.CALIB_FIX_INTRINSIC

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)
        if LooseVersion(cv2.__version__).version[0] == 2:
            cv2.stereoCalibrate(opts, lipts, ripts, self.size,
                                self.l.intrinsics, self.l.distortion,
                                self.r.intrinsics, self.r.distortion,
                                self.R,  # R
                                self.T,  # T
                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                                flags=flags)
        else:
            cv2.stereoCalibrate(opts, lipts, ripts,
                                self.l.intrinsics, self.l.distortion,
                                self.r.intrinsics, self.r.distortion,
                                self.size,
                                self.R,  # R
                                self.T,  # T
                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5),
                                flags=flags)

        self.set_alpha(0.0)

    def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution. The
        alpha value is a zoom, and ranges from 0 (zoomed in, all pixels
        in calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """

        cv2.stereoRectify(self.l.intrinsics,
                          self.l.distortion,
                          self.r.intrinsics,
                          self.r.distortion,
                          self.size,
                          self.R,
                          self.T,
                          self.l.R, self.r.R, self.l.P, self.r.P,
                          alpha=a)

        cv2.initUndistortRectifyMap(self.l.intrinsics, self.l.distortion, self.l.R, self.l.P, self.size, cv2.CV_32FC1,
                                    self.l.mapx, self.l.mapy)
        cv2.initUndistortRectifyMap(self.r.intrinsics, self.r.distortion, self.r.R, self.r.P, self.size, cv2.CV_32FC1,
                                    self.r.mapx, self.r.mapy)

    def as_message(self):
        """
        Return the camera calibration as a pair of CameraInfo messages, for left
        and right cameras respectively.
        """

        return (self.lrmsg(self.l.distortion, self.l.intrinsics, self.l.R, self.l.P),
                self.lrmsg(self.r.distortion, self.r.intrinsics, self.r.R, self.r.P))

    def from_message(self, msgs, alpha=0.0):
        """ Initialize the camera calibration from a pair of CameraInfo messages.  """
        self.size = (msgs[0].width, msgs[0].height)

        self.T = numpy.zeros((3, 1), dtype=numpy.float64)
        self.R = numpy.eye(3, dtype=numpy.float64)

        self.l.from_message(msgs[0])
        self.r.from_message(msgs[1])
        # Need to compute self.T and self.R here, using the monocular parameters above
        if False:
            self.set_alpha(0.0)

    def report(self):
        print("\nLeft:")
        self.lrreport(self.l.distortion, self.l.intrinsics, self.l.R, self.l.P)
        print("\nRight:")
        self.lrreport(self.r.distortion, self.r.intrinsics, self.r.R, self.r.P)
        print("self.T ", numpy.ravel(self.T).tolist())
        print("self.R ", numpy.ravel(self.R).tolist())

    def ost(self):
        return (self.lrost(self.name + "/left", self.l.distortion, self.l.intrinsics, self.l.R, self.l.P) +
                self.lrost(self.name + "/right", self.r.distortion, self.r.intrinsics, self.r.R, self.r.P))

    def yaml(self, suffix, info):
        return self.lryaml(self.name + suffix, info.distortion, info.intrinsics, info.R, info.P)

    # TODO Get rid of "from_images" versions of these, instead have function to get undistorted corners
    def epipolar_error_from_images(self, limage, rimage):
        """
        Detect the checkerboard in both images and compute the epipolar error.
        Mainly for use in tests.
        """
        lcorners = self.downsample_and_detect(limage)[1]
        rcorners = self.downsample_and_detect(rimage)[1]
        if lcorners is None or rcorners is None:
            return None

        lundistorted = self.l.undistort_points(lcorners)
        rundistorted = self.r.undistort_points(rcorners)

        return self.epipolar_error(lundistorted, rundistorted)

    def epipolar_error(self, lcorners, rcorners):
        """
        Compute the epipolar error from two sets of matching undistorted points
        """
        d = lcorners[:, :, 1] - rcorners[:, :, 1]
        return numpy.sqrt(numpy.square(d).sum() / d.size)

    def chessboard_size_from_images(self, limage, rimage):
        _, lcorners, _, board, _ = self.downsample_and_detect(limage)
        _, rcorners, _, board, _ = self.downsample_and_detect(rimage)
        if lcorners is None or rcorners is None:
            return None

        lundistorted = self.l.undistort_points(lcorners)
        rundistorted = self.r.undistort_points(rcorners)

        return self.chessboard_size(lundistorted, rundistorted, board)

    def chessboard_size(self, lcorners, rcorners, board, msg=None):
        """
        Compute the square edge length from two sets of matching undistorted points
        given the current calibration.
        :param msg: a tuple of (left_msg, right_msg)
        """
        # Project the points to 3d
        cam = image_geometry.StereoCameraModel()
        if msg == None:
            msg = self.as_message()
        cam.fromCameraInfo(*msg)
        disparities = lcorners[:, :, 0] - rcorners[:, :, 0]
        pt3d = [cam.projectPixelTo3d((lcorners[i, 0, 0], lcorners[i, 0, 1]), disparities[i, 0]) for i in
                range(lcorners.shape[0])]

        def l2(p0, p1):
            return math.sqrt(sum([(c0 - c1) ** 2 for (c0, c1) in zip(p0, p1)]))

        # Compute the length from each horizontal and vertical line, and return the mean
        cc = board.n_cols
        cr = board.n_rows
        lengths = (
            [l2(pt3d[cc * r + 0], pt3d[cc * r + (cc - 1)]) / (cc - 1) for r in range(cr)] +
            [l2(pt3d[c + 0], pt3d[c + (cc * (cr - 1))]) / (cr - 1) for c in range(cc)])
        return sum(lengths) / len(lengths)

    def handle_msg(self, msg):
        # TODO Various asserts that images have same dimension, same board detected...
        (lmsg, rmsg) = msg
        lgray = self.mkgray(lmsg)
        rgray = self.mkgray(rmsg)
        epierror = -1

        # Get display-images-to-be and detections of the calibration target
        lscrib_mono, lcorners, ldownsampled_corners, lboard, (x_scale, y_scale) = self.downsample_and_detect(lgray)
        rscrib_mono, rcorners, rdownsampled_corners, rboard, _ = self.downsample_and_detect(rgray)

        if self.calibrated:
            # Show rectified images
            lremap = self.l.remap(lgray)
            rremap = self.r.remap(rgray)
            lrect = lremap
            rrect = rremap
            if x_scale != 1.0 or y_scale != 1.0:
                lrect = cv2.resize(lremap, (lscrib_mono.shape[1], lscrib_mono.shape[0]))
                rrect = cv2.resize(rremap, (rscrib_mono.shape[1], rscrib_mono.shape[0]))

            lscrib = cv2.cvtColor(lrect, cv2.COLOR_GRAY2BGR)
            rscrib = cv2.cvtColor(rrect, cv2.COLOR_GRAY2BGR)

            # Draw rectified corners
            if lcorners is not None:
                lundistorted = self.l.undistort_points(lcorners)
                scrib_src = lundistorted.copy()
                scrib_src[:, :, 0] /= x_scale
                scrib_src[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(lscrib, (lboard.n_cols, lboard.n_rows), scrib_src, True)

            if rcorners is not None:
                rundistorted = self.r.undistort_points(rcorners)
                scrib_src = rundistorted.copy()
                scrib_src[:, :, 0] /= x_scale
                scrib_src[:, :, 1] /= y_scale
                cv2.drawChessboardCorners(rscrib, (rboard.n_cols, rboard.n_rows), scrib_src, True)

            # Report epipolar error
            if lcorners is not None and rcorners is not None and len(lcorners) == len(rcorners):
                epierror = self.epipolar_error(lundistorted, rundistorted)

        else:
            lscrib = cv2.cvtColor(lscrib_mono, cv2.COLOR_GRAY2BGR)
            rscrib = cv2.cvtColor(rscrib_mono, cv2.COLOR_GRAY2BGR)
            # Draw any detected chessboards onto display (downsampled) images
            if lcorners is not None:
                cv2.drawChessboardCorners(lscrib, (lboard.n_cols, lboard.n_rows),
                                          ldownsampled_corners, True)
            if rcorners is not None:
                cv2.drawChessboardCorners(rscrib, (rboard.n_cols, rboard.n_rows),
                                          rdownsampled_corners, True)

            # Add sample to database only if it's sufficiently different from any previous sample
            if lcorners is not None and rcorners is not None and len(lcorners) == len(rcorners):
                params = self.get_parameters(lcorners, lboard, (lgray.shape[1], lgray.shape[0]))
                if self.is_good_sample(params):
                    self.db.append((params, lgray, rgray))
                    self.good_corners.append((lcorners, rcorners, lboard))
                    print(("*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple(
                        [len(self.db)] + params)))

        rv = StereoDrawable()
        rv.lscrib = lscrib
        rv.rscrib = rscrib
        rv.params = self.compute_goodenough()
        rv.epierror = epierror
        return rv

    def do_calibration(self, dump=False):
        # TODO MonoCalibrator collects corners if needed here
        # Dump should only occur if user wants it
        if dump:
            pickle.dump((self.is_mono, self.size, self.good_corners),
                        open("camera_calibration_%08x.pickle" % random.getrandbits(32), "w"))
        self.size = (self.db[0][1].shape[1], self.db[0][1].shape[0])  # TODO Needs to be set externally
        self.l.size = self.size
        self.r.size = self.size
        self.cal_fromcorners(self.good_corners)
        self.calibrated = True
        # DEBUG
        print((self.report()))
        print((self.ost()))

    def do_tarfile_save(self, tf):
        """ Write images and calibration solution to a tarfile object """
        ims = ([("left-%04d.png" % i, im) for i, (_, im, _) in enumerate(self.db)] +
               [("right-%04d.png" % i, im) for i, (_, _, im) in enumerate(self.db)])

        def taradd(name, buf):
            if isinstance(buf, basestring):
                s = StringIO(buf)
            else:
                s = BytesIO(buf)
            ti = tarfile.TarInfo(name)
            ti.size = len(s.getvalue())
            ti.uname = 'calibrator'
            ti.mtime = int(time.time())
            tf.addfile(tarinfo=ti, fileobj=s)

        for (name, im) in ims:
            taradd(name, cv2.imencode(".png", im)[1].tostring())
        taradd('left.yaml', self.yaml("/left", self.l))
        taradd('right.yaml', self.yaml("/right", self.r))
        taradd('ost.txt', self.ost())

    def do_tarfile_calibration(self, filename):
        archive = tarfile.open(filename, 'r')
        limages = [image_from_archive(archive, f) for f in archive.getnames() if
                   (f.startswith('left') and (f.endswith('pgm') or f.endswith('png')))]
        rimages = [image_from_archive(archive, f) for f in archive.getnames() if
                   (f.startswith('right') and (f.endswith('pgm') or f.endswith('png')))]

        if not len(limages) == len(rimages):
            raise CalibrationException(
                "Left, right images don't match. %d left images, %d right" % (len(limages), len(rimages)))

        ##\todo Check that the filenames match and stuff

        self.cal(limages, rimages)
