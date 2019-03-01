#!/usr/bin/env python

from distutils.core import setup

from setuptools import find_packages
from catkin_pkg.python_setup import generate_distutils_setup


d = generate_distutils_setup(
    packages=find_packages('python'),
    package_dir={'': 'python'},
)

setup(**d)
