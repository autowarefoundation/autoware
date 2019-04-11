#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, shutil, shlex, subprocess
import numpy as np

class Colmap:
    def __init__(self, temporary_directory):
        self.temporary_directory = temporary_directory
        self.database_path = "{}/database.db".format(self.temporary_directory)
        self.images_path = "{}/images".format(self.temporary_directory)
        self.sparse_path = "{}/sparse".format(self.temporary_directory)
        self.sparse0_path = "{}/0".format(self.sparse_path)
        self.model_path = "{}/model".format(self.temporary_directory)

        self.camera_model = "OPENCV"
        self.max_num_features = 10000

    def _execute_command(self, cmd):
        print "EXECUTE: $ {}".format(cmd)
        return subprocess.check_output(shlex.split(cmd))

    def _sparse_reconstruction(self, image_paths):
        os.mkdir(self.images_path)
        os.mkdir(self.sparse_path)
        os.mkdir(self.model_path)

        for path in image_paths:
            shutil.move(path, self.images_path)

        feature_extractor_cmd = "colmap feature_extractor --ImageReader.single_camera 1 --ImageReader.camera_model {}  --SiftExtraction.max_num_features {} --database_path {} --image_path {}"
        feature_extractor_cmd = feature_extractor_cmd.format(self.camera_model, self.max_num_features, self.database_path, self.images_path)

        feature_matcher_cmd = "colmap exhaustive_matcher --database_path {}"
        feature_matcher_cmd = feature_matcher_cmd.format(self.database_path)

        sparse_reconstruction_cmd = "colmap mapper --database_path {} --output_path {} --image_path {}"
        sparse_reconstruction_cmd = sparse_reconstruction_cmd.format(self.database_path, self.sparse_path, self.images_path)

        self._execute_command(feature_extractor_cmd)
        self._execute_command(feature_matcher_cmd)
        self._execute_command(sparse_reconstruction_cmd)

        return os.path.exists(self.sparse0_path)

    def _get_parameters(self):
        model_converter_cmd = "colmap model_converter --input_path {} --output_type 'TXT' --output_path {}"
        model_converter_cmd = model_converter_cmd.format(self.sparse0_path, self.model_path)
        self._execute_command(model_converter_cmd)

        camera_txt_path = "{}/cameras.txt".format(self.model_path)
        if os.path.exists(camera_txt_path):
            try:
                with open(camera_txt_path) as cf:
                    parameters = cf.readlines()[3].split(" ")[2:]
                    camera_matrix = np.identity(3)
                    camera_matrix[0][0] = float(parameters[2])
                    camera_matrix[0][2] = float(parameters[4])
                    camera_matrix[1][1] = float(parameters[3])
                    camera_matrix[1][2] = float(parameters[5])
                    dist_coeffs = np.array([ float(x) for x in parameters[6:10] ])
                    return True, camera_matrix, dist_coeffs
            except Exception as e:
                return False, None, None
        else:
            return False, None, None

    def check_available(self):
        res = self._execute_command("colmap -h")
        return (res.split()[0] == "COLMAP")

    def execute_sfm(self, image_paths):
        success = self._sparse_reconstruction(image_paths)
        if success:
            success, camera_matrix, dist_coeffs = self._get_parameters()
            if success:
                return True, camera_matrix, dist_coeffs
            else:
                return False, None, None
        else:
            return False, None, None
