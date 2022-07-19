from setuptools import setup

package_name = "simulator_compatibility_test"
clients = "simulator_compatibility_test/clients/"
publishers = "simulator_compatibility_test/publishers/"
subscribers = "simulator_compatibility_test/subscribers/"

clients_moraisim = "simulator_compatibility_test/clients/moraisim/"
publishers_moraisim = "simulator_compatibility_test/publishers/moraisim/"

setup(
    name=package_name,
    version="0.0.0",
    packages=[
        package_name,
        clients,
        publishers,
        subscribers,
        clients_moraisim,
        publishers_moraisim,
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="shpark",
    maintainer_email="shpark@morai.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=[],
    entry_points={
        "console_scripts": [
            # Client
            (
                "morai_ctrl_mode"
                + "="
                + "simulator_compatibility_test"
                + ".clients.moraisim.morai_client_event_cmd:main"
            ),
            # Publisher
            (
                "gear_command"
                + "="
                + "simulator_compatibility_test"
                + ".publishers.gear_command:main"
            ),
            (
                "control_mode_command"
                + "="
                + "simulator_compatibility_test"
                + ".publishers.control_mode_command:main"
            ),
            (
                "morai_ctrl_cmd"
                + "="
                + "simulator_compatibility_test"
                + ".publishers.moraisim.morai_ctrl_cmd:main"
            ),
            # Subscriber
            (
                "gear_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.gear_report:main"
            ),
            (
                "control_mode_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.control_mode_report:main"
            ),
            (
                "velocity_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.velocity_report:main"
            ),
            (
                "steering_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.steering_report:main"
            ),
            (
                "turn_indicators_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.turn_indicators_report:main"
            ),
            (
                "hazard_lights_report"
                + "="
                + "simulator_compatibility_test"
                + ".subscribers.hazard_lights_report:main"
            ),
        ],
    },
)
