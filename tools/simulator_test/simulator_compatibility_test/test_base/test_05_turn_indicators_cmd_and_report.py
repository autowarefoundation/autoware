from enum import Enum
import time

from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from simulator_compatibility_test.subscribers.turn_indicators_report import (
    SubscriberTurnIndicatorsReport,
)
from simulator_compatibility_test.subscribers.turn_indicators_report import (
    TurnIndicatorsReport_Constants,
)


class TurnIndicatorsCommand_Constants(Enum):
    NO_COMMAND = 0
    DISABLE = 1
    ENABLE_LEFT = 2
    ENABLE_RIGHT = 3


class Test05TurnIndicatorsCmdAndReportBase:
    msgs_rx = []
    node = None
    sub = None
    pub = None
    sub_turn_indicators_status = None
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
        cls.node = rclpy.create_node("test_turn_indicator_cmd_and_report_base")
        cls.sub = cls.node.create_subscription(
            TurnIndicatorsCommand,
            "/control/command/turn_indicators_cmd",
            lambda msg: cls.msgs_rx.append(msg),
            10,
        )
        cls.pub = cls.node.create_publisher(
            TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", QOS_RKL10TL
        )
        cls.sub_turn_indicators_status = SubscriberTurnIndicatorsReport()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_turn_indicators_status)
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()

    def set_turn_indicators(self, command):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_turn_indicators_cmd_msg(command))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert received.command == command.value

    def generate_turn_indicators_cmd_msg(self, command):
        stamp = self.node.get_clock().now().to_msg()
        msg = TurnIndicatorsCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = command.value
        return msg

    def get_turn_indicators_status(self):
        time.sleep(1)
        self.sub_turn_indicators_status.received.clear()
        received = TurnIndicatorsReport_Constants.DISABLE
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_turn_indicators_status.received) > 2:
                    break
            received = self.sub_turn_indicators_status.received.pop()
        finally:
            return received
