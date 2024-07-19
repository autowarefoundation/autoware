from glob import glob
import os

from setuptools import setup

ROS_VERSION = int(os.environ["ROS_VERSION"])

package_name = "autoware_carla_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/" + package_name, glob("config/*")),
        ("share/" + package_name, glob("calibration_maps/*.csv")),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/autoware_carla_interface.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Muhammad Raditya GIOVANNI, Maxime CLEMENT",
    maintainer_email="mradityagio@gmail.com, maxime.clement@tier4.jp",
    description="CARLA ROS 2 bridge for AUTOWARE",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "autoware_carla_interface = autoware_carla_interface.carla_autoware:main"
        ],
    },
    package_dir={"": "src"},
)
