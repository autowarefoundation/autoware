#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2 as pc2
import cv2
from cv_bridge import CvBridge
import matplotlib.path as mplPath
from copy import deepcopy
import pcl
import numpy as np
from sklearn import mixture
from sklearn.decomposition import PCA
from scipy.optimize import minimize
import transforms3d

class LidarCameraCalibrator:
    def __init__(self):
        # Subscribers
        # clusterSubscriber = rospy.Subscriber("cloud_clusters", CloudClusterArray, self.points_callback)
        pointsSubscriber = rospy.Subscriber("points_raw", PointCloud2, self.points_callback)
        cameraSubscriber = rospy.Subscriber("image_raw", Image, self.image_callback)
        clickSubscriber = rospy.Subscriber("clicked_point", PointStamped, self.click_callback)

        # Publishers
        self.region_publisher = rospy.Publisher("/region_points", PointCloud2, queue_size=1)
        self.plane_publisher = rospy.Publisher("/plane_points", PointCloud2, queue_size=1)

        # Initialization
        self.bridge = CvBridge()
        self.image_data = []
        self.cluster_data = []
        self.clinsecscked_point = []

        self.marker_size = [6, 9]
        self.square_size = 0.1
        # self.chessboard_size = np.sqrt((self.square_size*self.marker_size[0])**2 +
        #                                (self.square_size*self.marker_size[1])**2)
        self.chessboard_size = 0.8
        self.start_pattern_corner = 0
        self.dist_thr = 0.01
        self.dist_w = 0.01

        rospy.spin()

    def image_callback(self, data):
        self.image_data = data

    def points_callback(self, data):
        self.points_data = data

    def click_callback(self, data):
        # Store data
        points = deepcopy(self.points_data)
        header = points.header
        image = deepcopy(self.cluster_data)

        # Image
        image_data = deepcopy(self.image_data)
        image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

        # Extract clicked point
        clicked_point = np.array([data.point.x, data.point.y, data.point.z])

        # Extract points within some radius
        region_points = []
        for point in pc2.read_points(points):
            point_array = np.array(point[0:3])
            if np.linalg.norm(clicked_point-point_array) < self.chessboard_size:
                region_points.append(list(point[0:4]))
        region_points_pcl = pcl.PointCloud_PointXYZI(region_points)

        # Get plane
        seg = region_points_pcl.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.dist_thr)
        seg.set_normal_distance_weight(self.dist_w)
        seg.set_max_iterations(200)
        indices, coefficients = seg.segment()
        # rospy.loginfo(coefficients)

        # Store plane points
        plane_points = np.zeros((len(indices), 4), dtype=np.float32)
        for i, index in enumerate(indices):
            plane_points[i][0] = region_points_pcl[index][0]
            plane_points[i][1] = region_points_pcl[index][1]
            plane_points[i][2] = region_points_pcl[index][2]
            plane_points[i][3] = region_points_pcl[index][3]
        plane = pcl.PointCloud_PointXYZI()
        plane.from_array(plane_points)

        plane_point = np.array([0, 0, -coefficients[3]/coefficients[2]])
        normal = np.array(coefficients[:3])
        projected_points_list = []
        for point in plane_points[:, :3]:
            vec = np.array(point) - np.array(plane_point)
            dist = np.dot(vec, normal)
            res = point - dist * normal
            projected_points_list.append(res)
        projected_points = np.array(projected_points_list, dtype=np.float32)
        stacked_points = np.hstack([projected_points, plane_points[:, 3:]])
        stacked_points_pcl = pcl.PointCloud_PointXYZI()
        stacked_points_pcl.from_array(plane_points)

        rot1, transed_pcd = self.transfer_by_pca(projected_points)
        t1 = transed_pcd.mean(axis=0)
        transed_pcd = transed_pcd - t1

        mean_array = np.array([[5], [60]])
        gmm = mixture.GaussianMixture(n_components=2, covariance_type="diag", max_iter=10000,
                                      means_init=mean_array).fit(stacked_points[:, 3:])
        # import matplotlib.pyplot as plt
        # plt.hist(stacked_points[:, 3:], 100)
        # plt.draw()
        # plt.pause(0.000001)
        print gmm.means_
        print gmm.covariances_

        if gmm.converged_:
            if gmm.means_[0] < gmm.means_[1]:
                low_intens = gmm.means_[0] + 2*gmm.covariances_[0]
                high_intens = gmm.means_[1] - 2*gmm.covariances_[0]
            else:
                high_intens = gmm.means_[0] + 2*gmm.covariances_[0]
                low_intens = gmm.means_[1] - 2*gmm.covariances_[0]
            if high_intens < low_intens:
                high_intens = gmm.means_.mean()
                low_intens = gmm.means_.mean()
            print low_intens
            print high_intens
        else:
            rospy.loginfo("GMM could not converge, try again")
        gray_zone = np.array([low_intens, high_intens])

        methods = ['Powell']
        res_dict = {}
        args = (transed_pcd, stacked_points, gray_zone,)
        param_ls = [[method, args] for method in methods]
        res_ls = map(self.opt_min, param_ls)
        for item in res_ls:
            if item is not None:
                res_dict[item[0]] = item[1]
        res = res_dict[min(res_dict)][1]
        print res_dict[min(res_dict)][0]
        print res

        rot2 = transforms3d.axangles.axangle2mat([0, 0, 1], res.x[0])
        t2 = np.array([res.x[1], res.x[2], 0])

        transed_pcd = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], res.x[0]),
                            (transed_pcd + np.array([[res.x[1], res.x[2], 0]])).T).T
        grid_coords = self.generate_grid_coords()
        grid_ls = [(p[0]).flatten()[:2] for p in grid_coords]
        corner_arr = np.transpose(np.array(grid_ls).reshape(self.marker_size[0], self.marker_size[1], 2)[1:, 1:],
                                  (1, 0, 2))
        t1 = t1.reshape(1, 3)
        t2 = t2.reshape(1, 3)
        corner_arr = corner_arr.reshape(-1, 2)
        num = corner_arr.shape[0]
        corner_arr = np.hstack([corner_arr, np.zeros(num).reshape(num, 1)])

        corners_in_pcd_arr = np.dot(np.dot(rot2.T, corner_arr.T).T - t2 + t1, rot1)
        corners_in_pcd_arr = np.array(corners_in_pcd_arr, dtype=np.float32)
        corners_pcl = pcl.PointCloud()
        corners_pcl.from_array(corners_in_pcd_arr)

        # Publish output to RVIZ
        fieldsXYZI = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                      pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                      pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                      pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1)]
        fieldsXYZ = [pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                     pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                     pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1)]
        # region_data = pc2.create_cloud(header, fieldsXYZI, region_points_pcl.to_list())
        # region_data = pc2.create_cloud(header, fieldsXYZI, stacked_points_pcl.to_list())
        region_data = pc2.create_cloud(header, fieldsXYZ, corners_pcl.to_list())
        self.region_publisher.publish(region_data)
        plane_data = pc2.create_cloud(header, fieldsXYZI, plane.to_list())
        self.plane_publisher.publish(plane_data)

    def opt_min(self, param_ls, initial_guess=np.zeros(3).tolist()):
        method = param_ls[0]
        try:
            res = minimize(self.cost_func_for_opt_mini, initial_guess, args=param_ls[1],
                           method=method, tol=1e-10, options={"maxiter": 10000000})  # , "disp": True

            print method, ": ", res.fun, "  ", res.x
            return res.fun, [method, res]
        except:
            print method, ": could not be applied"
            return None

    def transfer_by_pca(self, arr):
        pca = PCA(n_components=3)
        pca.fit(arr)
        trans_mat = pca.components_
        trans_mat[[0, 1]] = trans_mat[[1, 0]]
        trans_mat[2] = np.cross(trans_mat[0], trans_mat[1])
        sign2 = np.sign(np.dot(trans_mat[1], np.array([0, 0, 1])))
        trans_mat[[0, 1]] = sign2 * trans_mat[[0, 1]]
        sign = np.sign(np.dot(trans_mat[2], 0 - arr.mean(axis=0).T))
        trans_mat[[0, 2]] = sign * trans_mat[[0, 2]]
        tmp = np.dot(arr, trans_mat.T)
        # print pca.components_
        # print "x,y,cross", np.cross(pca.components_[1], pca.components_[2])
        return trans_mat, tmp


    def cost_func_for_opt_mini(self, theta_t, transed_pcd, marker_full_data_arr, gray_zone):
        x_res=self.marker_size[0]
        y_res=self.marker_size[1]
        grid_len=self.square_size

        transed_pcd_for_costf = np.dot(transforms3d.axangles.axangle2mat([0, 0, 1], theta_t[0]),
                                       (transed_pcd + np.array([[theta_t[1], theta_t[2], 0]])).T).T
        arr = np.hstack([transed_pcd_for_costf, marker_full_data_arr[:, 3:]])
        bound = np.array([[0, 0], [0, y_res], [x_res, y_res], [x_res, 0]]) * grid_len - np.array([x_res,
                                                                                         y_res]) * grid_len / 2

        x_grid_arr = (np.array(range(0, x_res + 1)) - float(x_res) / 2) * grid_len
        y_grid_arr = (np.array(range(0, y_res + 1)) - float(y_res) / 2) * grid_len

        x = range(arr.shape[0])
        y = []
        polygon_path = mplPath.Path(bound)
        cost = 0
        for row in arr:
            if polygon_path.contains_point(row[:2]):
                if gray_zone[0] < row[3] < gray_zone[1]:
                    y.append(0.5)
                    continue
                else:
                    i = int((row[0] + x_res * grid_len / 2) / grid_len)
                    j = int((row[1] + y_res * grid_len / 2) / grid_len)
                    if i % 2 == 0:
                        if j % 2 == 0:
                            color = 0
                        else:
                            color = 1
                    else:
                        if j % 2 == 0:
                            color = 1
                        else:
                            color = 0

                    estimated_color = (np.sign(row[3] - gray_zone[1]) + 1) / 2
                    if estimated_color != color:
                        cost += (min(abs(row[0] - x_grid_arr)) + min(abs(row[1] - y_grid_arr)))
                    y.append(color)
            else:
                cost += (min(abs(row[0] - x_grid_arr)) + min(abs(row[1] - y_grid_arr)))

                y.append(2)

        return cost

    def generate_grid_coords(self):  # res, resolution
        x_res=self.marker_size[0]
        y_res=self.marker_size[1]
        grid_len=self.square_size
        ls = []
        for i in xrange(x_res):
            for j in xrange(y_res):
                orig = np.array([i, j, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
                p1 = np.array([i + 1, j, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
                p2 = np.array([i, j + 1, 0]) * grid_len - np.array([x_res, y_res, 0]) * grid_len / 2
                if i % 2 == self.start_pattern_corner:
                    if j % 2 == self.start_pattern_corner:
                        color = self.start_pattern_corner
                    else:
                        color = 1 - self.start_pattern_corner
                else:
                    if j % 2 == self.start_pattern_corner:
                        color = 1 - self.start_pattern_corner
                    else:
                        color = self.start_pattern_corner
                ls.append([orig, p1, p2, color])
        return ls


if __name__=='__main__':
    calibration = LidarCameraCalibrator()
