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

from __future__ import print_function

import logging
from queue import Empty
from queue import Queue

import carla
import numpy as np

from .carla_data_provider import CarlaDataProvider


# Sensor Wrapper for Agent
class SensorReceivedNoData(Exception):
    """Exceptions when no data received from the sensors."""


class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame


class CallBack(object):
    def __init__(self, tag, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor)

    def __call__(self, data):
        if isinstance(data, carla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, carla.IMUMeasurement):
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudo_sensor(data, self._tag)
        else:
            logging.error("No callback method for this sensor.")

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        self._data_provider.update_sensor(tag, image, image.frame)

    def _parse_lidar_cb(self, lidar_data, tag):
        self._data_provider.update_sensor(tag, lidar_data, lidar_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        self._data_provider.update_sensor(tag, imu_data, imu_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array(
            [gnss_data.latitude, gnss_data.longitude, gnss_data.altitude], dtype=np.float64
        )
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_pseudo_sensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._new_data_buffers = Queue()
        self._queue_timeout = 10
        self.tag = ""

    def register_sensor(self, tag, sensor):
        self.tag = tag
        if tag in self._sensors_objects:
            raise ValueError(f"Duplicated sensor tag [{tag}]")

        self._sensors_objects[tag] = sensor

    def update_sensor(self, tag, data, timestamp):
        if tag not in self._sensors_objects:
            raise ValueError(f"Sensor with tag [{tag}] has not been created")

        self._new_data_buffers.put((tag, timestamp, data))

    def get_data(self):
        try:
            data_dict = {}
            while len(data_dict.keys()) < len(self._sensors_objects.keys()):
                sensor_data = self._new_data_buffers.get(True, self._queue_timeout)
                data_dict[sensor_data[0]] = (sensor_data[1], sensor_data[2])
        except Empty:
            raise SensorReceivedNoData(
                f"Sensor with tag [{self.tag}] took too long to send its data"
            )

        return data_dict


# Sensor Wrapper


class SensorWrapper(object):
    _agent = None
    _sensors_list = []

    def __init__(self, agent):
        self._agent = agent

    def __call__(self):
        return self._agent()

    def setup_sensors(self, vehicle, debug_mode=False):
        """Create and attach the sensor defined in objects.json."""
        bp_library = CarlaDataProvider.get_world().get_blueprint_library()

        for sensor_spec in self._agent.sensors["sensors"]:
            bp = bp_library.find(str(sensor_spec["type"]))

            if sensor_spec["type"].startswith("sensor.camera"):
                bp.set_attribute("image_size_x", str(sensor_spec["image_size_x"]))
                bp.set_attribute("image_size_y", str(sensor_spec["image_size_y"]))
                bp.set_attribute("fov", str(sensor_spec["fov"]))
                sensor_location = carla.Location(
                    x=sensor_spec["spawn_point"]["x"],
                    y=sensor_spec["spawn_point"]["y"],
                    z=sensor_spec["spawn_point"]["z"],
                )
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec["spawn_point"]["pitch"],
                    roll=sensor_spec["spawn_point"]["roll"],
                    yaw=sensor_spec["spawn_point"]["yaw"],
                )

            elif sensor_spec["type"].startswith("sensor.lidar"):
                bp.set_attribute("range", str(sensor_spec["range"]))
                bp.set_attribute("rotation_frequency", str(sensor_spec["rotation_frequency"]))
                bp.set_attribute("channels", str(sensor_spec["channels"]))
                bp.set_attribute("upper_fov", str(sensor_spec["upper_fov"]))
                bp.set_attribute("lower_fov", str(sensor_spec["lower_fov"]))
                bp.set_attribute("points_per_second", str(sensor_spec["points_per_second"]))
                sensor_location = carla.Location(
                    x=sensor_spec["spawn_point"]["x"],
                    y=sensor_spec["spawn_point"]["y"],
                    z=sensor_spec["spawn_point"]["z"],
                )
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec["spawn_point"]["pitch"],
                    roll=sensor_spec["spawn_point"]["roll"],
                    yaw=sensor_spec["spawn_point"]["yaw"],
                )

            elif sensor_spec["type"].startswith("sensor.other.gnss"):
                bp.set_attribute("noise_alt_stddev", str(0.0))
                bp.set_attribute("noise_lat_stddev", str(0.0))
                bp.set_attribute("noise_lon_stddev", str(0.0))
                bp.set_attribute("noise_alt_bias", str(0.0))
                bp.set_attribute("noise_lat_bias", str(0.0))
                bp.set_attribute("noise_lon_bias", str(0.0))
                sensor_location = carla.Location(
                    x=sensor_spec["spawn_point"]["x"],
                    y=sensor_spec["spawn_point"]["y"],
                    z=sensor_spec["spawn_point"]["z"],
                )
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec["spawn_point"]["pitch"],
                    roll=sensor_spec["spawn_point"]["roll"],
                    yaw=sensor_spec["spawn_point"]["yaw"],
                )

            elif sensor_spec["type"].startswith("sensor.other.imu"):
                bp.set_attribute("noise_accel_stddev_x", str(0.0))
                bp.set_attribute("noise_accel_stddev_y", str(0.0))
                bp.set_attribute("noise_accel_stddev_z", str(0.0))
                bp.set_attribute("noise_gyro_stddev_x", str(0.0))
                bp.set_attribute("noise_gyro_stddev_y", str(0.0))
                bp.set_attribute("noise_gyro_stddev_z", str(0.0))
                sensor_location = carla.Location(
                    x=sensor_spec["spawn_point"]["x"],
                    y=sensor_spec["spawn_point"]["y"],
                    z=sensor_spec["spawn_point"]["z"],
                )
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec["spawn_point"]["pitch"],
                    roll=sensor_spec["spawn_point"]["roll"],
                    yaw=sensor_spec["spawn_point"]["yaw"],
                )

            elif sensor_spec["type"].startswith("sensor.pseudo.*"):
                sensor_location = carla.Location(
                    x=sensor_spec["spawn_point"]["x"],
                    y=sensor_spec["spawn_point"]["y"],
                    z=sensor_spec["spawn_point"]["z"],
                )
                sensor_rotation = carla.Rotation(
                    pitch=sensor_spec["spawn_point"]["pitch"] + 0.001,
                    roll=sensor_spec["spawn_point"]["roll"] - 0.015,
                    yaw=sensor_spec["spawn_point"]["yaw"] + 0.0364,
                )

            # create sensor
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = CarlaDataProvider.get_world().spawn_actor(bp, sensor_transform, vehicle)
            # setup callback
            sensor.listen(CallBack(sensor_spec["id"], sensor, self._agent.sensor_interface))
            self._sensors_list.append(sensor)

        # Tick once to spawn the sensors
        CarlaDataProvider.get_world().tick()

    def cleanup(self):
        """Cleanup sensors."""
        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].stop()
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []
