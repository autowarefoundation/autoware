# Copyright 2024 Proxima Technology Inc, TIER IV
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

import os
from pathlib import Path
import shutil

import autoware_smart_mpc_trajectory_follower

if __name__ == "__main__":
    package_dir = str(Path(autoware_smart_mpc_trajectory_follower.__file__).parent)

    remove_dirs = [
        package_dir + "/__pycache__",
        package_dir + "/scripts/__pycache__",
        package_dir + "/training_and_data_check/__pycache__",
    ]
    for i in range(len(remove_dirs)):
        if os.path.isdir(remove_dirs[i]):
            shutil.rmtree(remove_dirs[i])
