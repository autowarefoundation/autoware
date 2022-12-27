from warnings import simplefilter

from pkg_resources import PkgResourcesDeprecationWarning
from setuptools import SetuptoolsDeprecationWarning
from setuptools import setup

simplefilter("ignore", category=SetuptoolsDeprecationWarning)
simplefilter("ignore", category=PkgResourcesDeprecationWarning)

package_name = "ad_api_visualizers"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [f"launch/{package_name}.launch.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Takagi, Isamu",
    maintainer_email="isamu.takagi@tier4.jp",
    description="The ad_api_visualizers package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["planning_factors = ad_api_visualizers.planning_factors:main"],
    },
)
