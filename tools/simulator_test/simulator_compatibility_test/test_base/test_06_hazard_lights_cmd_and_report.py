from enum import Enum
import time

from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from simulator_compatibility_test.subscribers.hazard_lights_report import (
    HazardLightsReport_Constants,
)
from simulator_compatibility_test.subscribers.hazard_lights_report import (
    SubscriberHazardLightsReport,
)


class HazardLightsCommand_Constants(Enum):
    NO_COMMAND = 0
    DISABLE = 1
    ENABLE = 2


class Test06HazardLightsCmdAndReportBase:
    msgs_rx = []
    node = None
    sub = None
    pub = None
    sub_hazard_lights_status = None
    executor = None

    @classmethod
    def setup_class(cls) -> None:
        QOS_RKL10TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        rclpy.init()
        cls.msgs_rx = []
        cls.node = rclpy.create_node("test_hazard_lights_cmd_and_report_base")
        cls.sub = cls.node.create_subscription(
            HazardLightsCommand,
            "/control/command/hazard_lights_cmd",
            lambda msg: cls.msgs_rx.append(msg),
            10,
        )
        cls.pub = cls.node.create_publisher(
            HazardLightsCommand, "/control/command/hazard_lights_cmd", QOS_RKL10TL
        )
        cls.sub_hazard_lights_status = SubscriberHazardLightsReport()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_hazard_lights_status)
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()

    def set_hazard_lights(self, command):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_hazard_lights_cmd_msg(command))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert received.command == command.value

    def generate_hazard_lights_cmd_msg(self, command):
        stamp = self.node.get_clock().now().to_msg()
        msg = HazardLightsCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = command.value
        return msg

    def get_hazard_lights_status(self):
        time.sleep(1)
        self.sub_hazard_lights_status.received.clear()
        received = HazardLightsReport_Constants.DISABLE
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_hazard_lights_status.received) > 2:
                    break
            received = self.sub_hazard_lights_status.received.pop()
        finally:
            return received
