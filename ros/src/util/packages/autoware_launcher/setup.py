from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    package_dir={"autoware_launcher": "src/autoware_launcher"},
    packages=[
        "autoware_launcher",
        "autoware_launcher.tool",
        "autoware_launcher.core",
        "autoware_launcher.gui",
        "autoware_launcher.gui.simulation",
        "autoware_launcher.gui.plugins",
    ],
)

setup(**setup_args)
