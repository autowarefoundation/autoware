#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['jsk_rviz_plugins'],
    package_dir={'': 'python'},
)

setup(**d)
