# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # use_sim_time
    set_use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    # vehicle_info
    vehicle_description_pkg = FindPackageShare(
        [LaunchConfiguration("vehicle_model"), "_description"]
    ).perform(context)

    load_vehicle_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vehicle_info_util"), "/launch/vehicle_info.launch.py"]
        ),
        launch_arguments={
            "vehicle_info_param_file": [vehicle_description_pkg, "/config/vehicle_info.param.yaml"]
        }.items(),
    )

    return [
        set_use_sim_time,
        load_vehicle_info,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
