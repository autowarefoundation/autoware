import numpy
import yaml
import cv2
from collections import OrderedDict


class YamlMaker:
    def __init__(self, camera_name, distortion, intrinsics, size):
        self.camera_name = camera_name
        self.distortion = distortion
        self.intrinsics = intrinsics
        self.size = size

    def opencv_matrix_constructor(self, loader, node):
        mapping = loader.construct_mapping(node, deep=True)
        mat = numpy.array(mapping["data"])
        mat.resize(mapping["rows"], mapping["cols"])
        return mat

    def opencv_matrix_representer(self, dumper, mat):
        mapping = OrderedDict()
        mapping['row'] = mat.shape[0]
        mapping['cols'] = mat.shape[1]
        mapping['dt'] = 'd'
        mapping['data'] = mat.reshape(-1).tolist()
        # mapping = {'rows': mat.shape[0], 'cols': mat.shape[1], 'dt': 'd', 'data': mat.reshape(-1).tolist()}
        return dumper.represent_mapping(u"tag:yaml.org,2002:opencv-matrix", mapping)

    def do_autoware_yaml(self):
        yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", self.opencv_matrix_constructor)
        yaml.add_representer(numpy.ndarray, self.opencv_matrix_representer)
        # camera_name = self.name
        distortion = self.distortion
        intrinsics = self.intrinsics
        reproj_error = 0.0
        # self.R, self.P
        fn = self.camera_name + '.yaml'
        with open(fn, 'w') as f:
            f.write("%YAML:1.0\n")
            yaml.dump({"CameraExtrinsics": numpy.ones((4,4)),
                       "CameraMat": intrinsics,
                       "DistCoeff": distortion}, f)
            f.write("ImageSize: [" + str(self.size[0]) + ", " + str(self.size[1]) + "]\n")
            f.write("Reprojection Error: " + str(reproj_error))

calib = YamlMaker('cam1', numpy.zeros((1,5)), numpy.eye(4), [1080, 720])
calib.do_autoware_yaml()