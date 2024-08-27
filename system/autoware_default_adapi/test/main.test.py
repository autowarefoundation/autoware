# Copyright 2022 TIER IV, Inc.
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

import importlib.util
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing.actions
import launch_testing.markers
import launch_testing.tools
import pytest


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    path = get_package_share_directory("autoware_default_adapi") + "/launch/default_adapi.launch.py"
    specification = importlib.util.spec_from_file_location("launch_script", path)
    launch_script = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(launch_script)
    return launch.LaunchDescription(
        [
            *launch_script.generate_launch_description().describe_sub_entities(),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMain(unittest.TestCase):
    def test_interface_version(self, launch_service, proc_info, proc_output):
        prefix = get_package_share_directory("autoware_default_adapi")
        target = prefix + "/test/node/interface_version.py"
        action = launch.actions.ExecuteProcess(cmd=["python3", target])
        with launch_testing.tools.launch_process(launch_service, action, proc_info, proc_output):
            proc_info.assertWaitForStartup(process=action, timeout=1)
            proc_info.assertWaitForShutdown(process=action, timeout=3)
        launch_testing.asserts.assertExitCodes(proc_info, process=action)
