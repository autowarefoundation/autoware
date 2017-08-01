################################################################################
#
# Copyright (c) 2017 University of Oxford
# Authors:
#  Geoff Pascoe (gmp@robots.ox.ac.uk)
#
# This work is licensed under the Creative Commons
# Attribution-NonCommercial-ShareAlike 4.0 International License.
# To view a copy of this license, visit
# http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to
# Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
#
################################################################################

import re
import os
import numpy as np
import scipy.interpolate as interp
from scipy.ndimage import map_coordinates


class CameraModel:
    """Provides intrinsic parameters and undistortion LUT for a camera.

    Attributes:
        camera (str): Name of the camera.
        camera sensor (str): Name of the sensor on the camera for multi-sensor cameras.
        focal_length (tuple[float]): Focal length of the camera in horizontal and vertical axis, in pixels.
        principal_point (tuple[float]): Principal point of camera for pinhole projection model, in pixels.
        G_camera_image (:obj: `numpy.matrixlib.defmatrix.matrix`): Transform from image frame to camera frame.
        bilinear_lut (:obj: `numpy.ndarray`): Look-up table for undistortion of images, mapping pixels in an undistorted
            image to pixels in the distorted image

    """
    
    cam_stereo_center = 1
    cam_stereo_right = 2
    cam_stereo_left = 3

    def __init__(self, models_dir, camera_type):
        """Loads a camera model from disk.

        Args:
            models_dir (str): directory containing camera model files.
            images_dir (str): directory containing images for which to read camera model.

        """
        self.camera = None
        self.camera_sensor = None
        self.focal_length = None
        self.principal_point = None
        self.G_camera_image = None
        self.bilinear_lut = None

        self.__load_intrinsics(models_dir, camera_type)
        self.__load_lut(models_dir, camera_type)

    def project(self, xyz, image_size):
        """Projects a pointcloud into the camera using a pinhole camera model.

        Args:
            xyz (:obj: `numpy.ndarray`): 3xn array, where each column is (x, y, z) point relative to camera frame.
            image_size (tuple[int]): dimensions of image in pixels

        Returns:
            numpy.ndarray: 2xm array of points, where each column is the (u, v) pixel coordinates of a point in pixels.
            numpy.array: array of depth values for points in image.

        Note:
            Number of output points m will be less than or equal to number of input points n, as points that do not
            project into the image are discarded.

        """
        if xyz.shape[0] == 3:
            xyz = np.stack((xyz, np.ones((1, xyz.shape[1]))))
        xyzw = np.linalg.solve(self.G_camera_image, xyz)

        # Find which points lie in front of the camera
        in_front = [i for i in range(0, xyzw.shape[1]) if xyzw[2, i] >= 0]
        xyzw = xyzw[:, in_front]

        uv = np.vstack((self.focal_length[0] * xyzw[0, :] / xyzw[2, :] + self.principal_point[0],
                        self.focal_length[1] * xyzw[1, :] / xyzw[2, :] + self.principal_point[1]))

        in_img = [i for i in range(0, uv.shape[1])
                  if 0.5 <= uv[0, i] <= image_size[1] and 0.5 <= uv[1, i] <= image_size[0]]

        return uv[:, in_img], np.ravel(xyzw[2, in_img])

    def undistort(self, image):
        """Undistorts an image.

        Args:
            image (:obj: `numpy.ndarray`): A distorted image. Must be demosaiced - ie. must be a 3-channel RGB image.

        Returns:
            numpy.ndarray: Undistorted version of image.

        Raises:
            ValueError: if image size does not match camera model.
            ValueError: if image only has a single channel.

        """
        if image.shape[0] * image.shape[1] != self.bilinear_lut.shape[0]:
            raise ValueError('Incorrect image size for camera model')

        lut = self.bilinear_lut[:, 1::-1].T.reshape((2, image.shape[0], image.shape[1]))

        if len(image.shape) == 1:
            raise ValueError('Undistortion function only works with multi-channel images')

        undistorted = np.rollaxis(np.array([map_coordinates(image[:, :, channel], lut, order=1)
                                for channel in range(0, image.shape[2])]), 0, 3)

        return undistorted.astype(image.dtype)

    @staticmethod
    def __get_model_name(camera_type):
        if camera_type == CameraModel.cam_stereo_center:
            return 'stereo_narrow_left'
        elif camera_type == CameraModel.cam_stereo_left:
            return 'stereo_wide_left'
        elif camera_type == CameraModel.cam_stereo_right:
            return 'stereo_wide_right'
        else:
            raise RuntimeError("Unknown camera type")

    def __load_intrinsics(self, models_dir, camera_type):
        model_name = CameraModel.__get_model_name(camera_type)
        intrinsics_path = os.path.join(models_dir, model_name + '.txt')

        with open(intrinsics_path) as intrinsics_file:
            vals = [float(x) for x in next(intrinsics_file).split()]
            self.focal_length = (vals[0], vals[1])
            self.principal_point = (vals[2], vals[3])

            G_camera_image = []
            for line in intrinsics_file:
                G_camera_image.append([float(x) for x in line.split()])
            self.G_camera_image = np.array(G_camera_image)

    def __load_lut(self, models_dir, camera_type):
        model_name = CameraModel.__get_model_name(camera_type)
        lut_path = os.path.join(models_dir, model_name + '_distortion_lut.bin')

        lut = np.fromfile(lut_path, np.double)
        lut = lut.reshape([2, lut.size // 2])
        self.bilinear_lut = lut.transpose()

