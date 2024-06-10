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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
import yaml


def launch_setup(context, *args, **kwargs):
    vehicle_param_file = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_param_file, "r") as f:
        vehicle_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    return [SetParameter(name=k, value=v) for (k, v) in vehicle_param.items()]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_info_param_file"),
            OpaqueFunction(function=launch_setup),
        ]
    )
