# Copyright 2021 The Autoware Foundation.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ament_index_python
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

import os


def generate_launch_description():

    default_vehicle_characteristics_param = os.path.join(
        get_package_share_directory('simple_planning_simulator'),
        'param/vehicle_characteristics.param.yaml')

    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=default_vehicle_characteristics_param,
        description='Path to config file for vehicle characteristics'
    )

    default_vehicle_info_param = os.path.join(
        get_package_share_directory('vehicle_info_util'),
        'config/vehicle_info.param.yaml')

    vehicle_info_param = DeclareLaunchArgument(
        'vehicle_info_param_file',
        default_value=default_vehicle_info_param,
        description='Path to config file for vehicle information'
    )

    simple_planning_simulator_node = launch_ros.actions.Node(
        package='simple_planning_simulator',
        executable='simple_planning_simulator_exe',
        name='simple_planning_simulator',
        namespace='simulation',
        output='screen',
        parameters=[
            "{}/param/simple_planning_simulator_default.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "simple_planning_simulator"
                )
            ),
            LaunchConfiguration('vehicle_characteristics_param_file'),
            LaunchConfiguration('vehicle_info_param_file')
        ],
        remappings=[
            ('input/ackermann_control_command', '/control/command/control_cmd'),
            ('input/gear_command', '/control/command/gear_cmd'),
            ('input/turn_indicators_command', '/control/command/turn_indicators_cmd'),
            ('input/hazard_lights_command', '/control/command/hazard_lights_cmd'),
            ('input/trajectory', '/planning/scenario_planning/trajectory'),
            ('output/twist', '/vehicle/status/velocity_status'),
            ('output/odometry', '/localization/kinematic_state'),
            ('output/steering', '/vehicle/status/steering_status'),
            ('output/gear_report', '/vehicle/status/gear_status'),
            ('output/turn_indicators_report', '/vehicle/status/turn_indicators_status'),
            ('output/hazard_lights_report', '/vehicle/status/hazard_lights_status'),
            ('output/control_mode_report', '/vehicle/status/control_mode'),
            ('/initialpose', '/initialpose'),
        ]
    )

    map_to_odom_tf_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_tf_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'])

    ld = launch.LaunchDescription([
        vehicle_characteristics_param,
        vehicle_info_param,
        simple_planning_simulator_node,
        map_to_odom_tf_publisher
    ])
    return ld
