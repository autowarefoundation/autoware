from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['libjavad_navsat_driver'],
    package_dir={'': 'lib'}
)

setup(**d)
