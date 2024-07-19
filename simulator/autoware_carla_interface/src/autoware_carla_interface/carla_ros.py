# Copyright 2024 Tier IV, Inc.
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
# limitations under the License.sr/bin/env python

import json
import math
import threading

from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time
import carla
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from tier4_vehicle_msgs.msg import ActuationCommandStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from transforms3d.euler import euler2quat

from .modules.carla_data_provider import GameTime
from .modules.carla_data_provider import datetime
from .modules.carla_utils import carla_location_to_ros_point
from .modules.carla_utils import carla_rotation_to_ros_quaternion
from .modules.carla_utils import create_cloud
from .modules.carla_utils import ros_pose_to_carla_transform
from .modules.carla_wrapper import SensorInterface


class carla_ros2_interface(object):
    def __init__(self):
        self.sensor_interface = SensorInterface()
        self.timestamp = None
        self.ego_actor = None
        self.physics_control = None
        self.channels = 0
        self.id_to_sensor_type_map = {}
        self.id_to_camera_info_map = {}
        self.cv_bridge = CvBridge()
        self.first_ = True
        self.pub_lidar = {}
        self.sensor_frequencies = {
            "top": 11,
            "left": 11,
            "right": 11,
            "camera": 11,
            "imu": 50,
            "status": 50,
            "pose": 2,
        }
        self.publish_prev_times = {
            sensor: datetime.datetime.now() for sensor in self.sensor_frequencies
        }

        # initialize ros2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("carla_ros2_interface")
        self.parameters = {
            "host": rclpy.Parameter.Type.STRING,
            "port": rclpy.Parameter.Type.INTEGER,
            "sync_mode": rclpy.Parameter.Type.BOOL,
            "timeout": rclpy.Parameter.Type.INTEGER,
            "fixed_delta_seconds": rclpy.Parameter.Type.DOUBLE,
            "carla_map": rclpy.Parameter.Type.STRING,
            "ego_vehicle_role_name": rclpy.Parameter.Type.STRING,
            "spawn_point": rclpy.Parameter.Type.STRING,
            "vehicle_type": rclpy.Parameter.Type.STRING,
            "objects_definition_file": rclpy.Parameter.Type.STRING,
            "use_traffic_manager": rclpy.Parameter.Type.BOOL,
            "max_real_delta_seconds": rclpy.Parameter.Type.DOUBLE,
        }
        self.param_values = {}
        for param_name, param_type in self.parameters.items():
            self.ros2_node.declare_parameter(param_name, param_type)
            self.param_values[param_name] = self.ros2_node.get_parameter(param_name).value

        # Publish clock
        self.clock_publisher = self.ros2_node.create_publisher(Clock, "/clock", 10)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=0)
        self.clock_publisher.publish(obj_clock)

        # Sensor Config (Edit your sensor here)
        self.sensors = json.load(open(self.param_values["objects_definition_file"]))

        # Subscribing Autoware Control messages and converting to CARLA control
        self.sub_control = self.ros2_node.create_subscription(
            ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        )

        self.sub_vehicle_initialpose = self.ros2_node.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initialpose_callback, 1
        )

        self.current_control = carla.VehicleControl()

        # Direct data publishing from CARLA for Autoware
        self.pub_pose_with_cov = self.ros2_node.create_publisher(
            PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", 1
        )
        self.pub_vel_state = self.ros2_node.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )
        self.pub_steering_state = self.ros2_node.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.pub_ctrl_mode = self.ros2_node.create_publisher(
            ControlModeReport, "/vehicle/status/control_mode", 1
        )
        self.pub_gear_state = self.ros2_node.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        self.pub_actuation_status = self.ros2_node.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", 1
        )

        # Create Publisher for each Physical Sensors
        for sensor in self.sensors["sensors"]:
            self.id_to_sensor_type_map[sensor["id"]] = sensor["type"]
            if sensor["type"] == "sensor.camera.rgb":
                self.pub_camera = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/traffic_light/image_raw", 1
                )
                self.pub_camera_info = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/traffic_light/camera_info", 1
                )
            elif sensor["type"] == "sensor.lidar.ray_cast":
                if sensor["id"] in self.sensor_frequencies:
                    self.pub_lidar[sensor["id"]] = self.ros2_node.create_publisher(
                        PointCloud2, f'/sensing/lidar/{sensor["id"]}/pointcloud', 10
                    )
                else:
                    self.ros2_node.get_logger().info(
                        "Please use Top, Right, or Left as the LIDAR ID"
                    )
            elif sensor["type"] == "sensor.other.imu":
                self.pub_imu = self.ros2_node.create_publisher(
                    Imu, "/sensing/imu/tamagawa/imu_raw", 1
                )
            else:
                self.ros2_node.get_logger().info(f'No Publisher for {sensor["type"]} Sensor')
                pass

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node,))
        self.spin_thread.start()

    def __call__(self):
        input_data = self.sensor_interface.get_data()
        timestamp = GameTime.get_time()
        control = self.run_step(input_data, timestamp)
        return control

    def get_param(self):
        return self.param_values

    def checkFrequency(self, sensor):
        time_delta = (
            datetime.datetime.now() - self.publish_prev_times[sensor]
        ).microseconds / 1000000.0
        if 1.0 / time_delta >= self.sensor_frequencies[sensor]:
            return True
        return False

    def get_msg_header(self, frame_id):
        """Obtain and modify ROS message header."""
        header = Header()
        header.frame_id = frame_id
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        header.stamp = Time(sec=seconds, nanosec=nanoseconds)
        return header

    def lidar(self, carla_lidar_measurement, id_):
        """Transform the received lidar measurement into a ROS point cloud message."""
        if self.checkFrequency(id_):
            return
        self.publish_prev_times[id_] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="velodyne_top_changed")
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="return_type", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="channel", offset=14, datatype=PointField.UINT16, count=1),
        ]

        lidar_data = numpy.frombuffer(
            carla_lidar_measurement.raw_data, dtype=numpy.float32
        ).reshape(-1, 4)
        intensity = lidar_data[:, 3]
        intensity = (
            numpy.clip(intensity, 0, 1) * 255
        )  # CARLA lidar intensity values are between 0 and 1
        intensity = intensity.astype(numpy.uint8).reshape(-1, 1)

        return_type = numpy.zeros((lidar_data.shape[0], 1), dtype=numpy.uint8)
        channel = numpy.empty((0, 1), dtype=numpy.uint16)
        self.channels = self.sensors["sensors"]

        for i in range(self.channels[1]["channels"]):
            current_ring_points_count = carla_lidar_measurement.get_point_count(i)
            channel = numpy.vstack(
                (channel, numpy.full((current_ring_points_count, 1), i, dtype=numpy.uint16))
            )

        lidar_data = numpy.hstack((lidar_data[:, :3], intensity, return_type, channel))
        lidar_data[:, 1] *= -1

        dtype = [
            ("x", "f4"),
            ("y", "f4"),
            ("z", "f4"),
            ("intensity", "u1"),
            ("return_type", "u1"),
            ("channel", "u2"),
        ]

        structured_lidar_data = numpy.zeros(lidar_data.shape[0], dtype=dtype)
        structured_lidar_data["x"] = lidar_data[:, 0]
        structured_lidar_data["y"] = lidar_data[:, 1]
        structured_lidar_data["z"] = lidar_data[:, 2]
        structured_lidar_data["intensity"] = lidar_data[:, 3].astype(numpy.uint8)
        structured_lidar_data["return_type"] = lidar_data[:, 4].astype(numpy.uint8)
        structured_lidar_data["channel"] = lidar_data[:, 5].astype(numpy.uint16)

        point_cloud_msg = create_cloud(header, fields, structured_lidar_data)
        self.pub_lidar[id_].publish(point_cloud_msg)

    def initialpose_callback(self, data):
        """Transform RVIZ initial pose to CARLA."""
        pose = data.pose.pose
        pose.position.z += 2.0
        carla_pose_transform = ros_pose_to_carla_transform(pose)
        if self.ego_actor is not None:
            self.ego_actor.set_transform(carla_pose_transform)
        else:
            print("Can't find Ego Vehicle")

    def pose(self):
        """Transform odometry data to Pose and publish Pose with Covariance message."""
        if self.checkFrequency("pose"):
            return
        self.publish_prev_times["pose"] = datetime.datetime.now()

        header = self.get_msg_header(frame_id="map")
        out_pose_with_cov = PoseWithCovarianceStamped()
        pose_carla = Pose()
        pose_carla.position = carla_location_to_ros_point(self.ego_actor.get_transform().location)
        pose_carla.orientation = carla_rotation_to_ros_quaternion(
            self.ego_actor.get_transform().rotation
        )
        out_pose_with_cov.header = header
        out_pose_with_cov.pose.pose = pose_carla
        out_pose_with_cov.pose.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_pose_with_cov.publish(out_pose_with_cov)

    def _build_camera_info(self, camera_actor):
        """Build camera info."""
        camera_info = CameraInfo()
        camera_info.width = camera_actor.width
        camera_info.height = camera_actor.height
        camera_info.distortion_model = "plumb_bob"
        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (2.0 * math.tan(camera_actor.fov * math.pi / 360.0))
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._camera_info = camera_info

    def camera(self, carla_camera_data):
        """Transform the received carla camera data into a ROS image and info message and publish."""
        while self.first_:
            self._camera_info_ = self._build_camera_info(carla_camera_data)
            self.first_ = False

        if self.checkFrequency("camera"):
            return
        self.publish_prev_times["camera"] = datetime.datetime.now()

        image_data_array = numpy.ndarray(
            shape=(carla_camera_data.height, carla_camera_data.width, 4),
            dtype=numpy.uint8,
            buffer=carla_camera_data.raw_data,
        )
        # cspell:ignore interp bgra
        img_msg = self.cv_bridge.cv2_to_imgmsg(image_data_array, encoding="bgra8")
        img_msg.header = self.get_msg_header(
            frame_id="traffic_light_left_camera/camera_optical_link"
        )
        cam_info = self._camera_info
        cam_info.header = img_msg.header
        self.pub_camera_info.publish(cam_info)
        self.pub_camera.publish(img_msg)

    def imu(self, carla_imu_measurement):
        """Transform a received imu measurement into a ROS Imu message and publish Imu message."""
        if self.checkFrequency("imu"):
            return
        self.publish_prev_times["imu"] = datetime.datetime.now()

        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(frame_id="tamagawa/imu_link_changed")
        imu_msg.angular_velocity.x = -carla_imu_measurement.gyroscope.x
        imu_msg.angular_velocity.y = carla_imu_measurement.gyroscope.y
        imu_msg.angular_velocity.z = -carla_imu_measurement.gyroscope.z

        imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
        imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
        imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z

        roll = math.radians(carla_imu_measurement.transform.rotation.roll)
        pitch = -math.radians(carla_imu_measurement.transform.rotation.pitch)
        yaw = -math.radians(carla_imu_measurement.transform.rotation.yaw)

        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        self.pub_imu.publish(imu_msg)

    def control_callback(self, in_cmd):
        """Convert and publish CARLA Ego Vehicle Control to AUTOWARE."""
        out_cmd = carla.VehicleControl()
        out_cmd.throttle = in_cmd.actuation.accel_cmd
        # convert base on steer curve of the vehicle
        steer_curve = self.physics_control.steering_curve
        current_vel = self.ego_actor.get_velocity()
        max_steer_ratio = numpy.interp(
            abs(current_vel.x), [v.x for v in steer_curve], [v.y for v in steer_curve]
        )
        out_cmd.steer = (
            -in_cmd.actuation.steer_cmd
            * max_steer_ratio
            * math.radians(self.physics_control.wheels[0].max_steer_angle)
        )
        out_cmd.brake = in_cmd.actuation.brake_cmd
        self.current_control = out_cmd

    def ego_status(self):
        """Publish ego vehicle status."""
        if self.checkFrequency("status"):
            return

        self.publish_prev_times["status"] = datetime.datetime.now()

        # convert velocity from cartesian to ego frame
        trans_mat = numpy.array(self.ego_actor.get_transform().get_matrix()).reshape(4, 4)
        rot_mat = trans_mat[0:3, 0:3]
        inv_rot_mat = rot_mat.T
        vel_vec = numpy.array(
            [
                self.ego_actor.get_velocity().x,
                self.ego_actor.get_velocity().y,
                self.ego_actor.get_velocity().z,
            ]
        ).reshape(3, 1)
        ego_velocity = (inv_rot_mat @ vel_vec).T[0]

        out_vel_state = VelocityReport()
        out_steering_state = SteeringReport()
        out_ctrl_mode = ControlModeReport()
        out_gear_state = GearReport()
        out_actuation_status = ActuationStatusStamped()

        out_vel_state.header = self.get_msg_header(frame_id="base_link")
        out_vel_state.longitudinal_velocity = ego_velocity[0]
        out_vel_state.lateral_velocity = ego_velocity[1]
        out_vel_state.heading_rate = (
            self.ego_actor.get_transform().transform_vector(self.ego_actor.get_angular_velocity()).z
        )

        out_steering_state.stamp = out_vel_state.header.stamp
        out_steering_state.steering_tire_angle = -math.radians(
            self.ego_actor.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        )

        out_gear_state.stamp = out_vel_state.header.stamp
        out_gear_state.report = GearReport.DRIVE

        out_ctrl_mode.stamp = out_vel_state.header.stamp
        out_ctrl_mode.mode = ControlModeReport.AUTONOMOUS

        control = self.ego_actor.get_control()
        out_actuation_status.header = self.get_msg_header(frame_id="base_link")
        out_actuation_status.status.accel_status = control.throttle
        out_actuation_status.status.brake_status = control.brake
        out_actuation_status.status.steer_status = -control.steer

        self.pub_actuation_status.publish(out_actuation_status)
        self.pub_vel_state.publish(out_vel_state)
        self.pub_steering_state.publish(out_steering_state)
        self.pub_ctrl_mode.publish(out_ctrl_mode)
        self.pub_gear_state.publish(out_gear_state)

    def run_step(self, input_data, timestamp):
        self.timestamp = timestamp
        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        obj_clock = Clock()
        obj_clock.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock_publisher.publish(obj_clock)

        # publish data of all sensors
        for key, data in input_data.items():
            sensor_type = self.id_to_sensor_type_map[key]
            if sensor_type == "sensor.camera.rgb":
                self.camera(data[1])
            elif sensor_type == "sensor.other.gnss":
                self.pose()
            elif sensor_type == "sensor.lidar.ray_cast":
                self.lidar(data[1], key)
            elif sensor_type == "sensor.other.imu":
                self.imu(data[1])
            else:
                self.ros2_node.get_logger().info("No Publisher for [{key}] Sensor")

        # Publish ego vehicle status
        self.ego_status()
        return self.current_control

    def shutdown(self):
        self.ros2_node.destroy_node()
