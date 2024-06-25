# cspell: ignore numba
import glob
import json
import os
from pathlib import Path

from setuptools import find_packages
from setuptools import setup

os.system("pip3 install numba==0.58.1 --force-reinstall")
os.system("pip3 install pybind11")
os.system("pip3 install GPy")
os.system("pip3 install torch")
package_path = {}
package_path["path"] = str(Path(__file__).parent)
with open("autoware_smart_mpc_trajectory_follower/package_path.json", "w") as f:
    json.dump(package_path, f)
build_cpp_command = "g++ -Ofast -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) "
build_cpp_command += "autoware_smart_mpc_trajectory_follower/scripts/proxima_calc.cpp "
build_cpp_command += "-o autoware_smart_mpc_trajectory_follower/scripts/proxima_calc$(python3-config --extension-suffix) "
build_cpp_command += "-lrt -I/usr/include/eigen3"
os.system(build_cpp_command)

so_path = (
    "scripts/"
    + glob.glob("autoware_smart_mpc_trajectory_follower/scripts/proxima_calc.*.so")[0].split("/")[
        -1
    ]
)
setup(
    name="autoware_smart_mpc_trajectory_follower",
    version="1.0.0",
    packages=find_packages(),
    package_data={
        "autoware_smart_mpc_trajectory_follower": ["package_path.json", so_path],
    },
)
