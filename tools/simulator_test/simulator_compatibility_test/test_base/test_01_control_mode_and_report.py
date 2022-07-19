from enum import Enum
import time

from autoware_auto_vehicle_msgs.msg import ControlModeCommand
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from simulator_compatibility_test.subscribers.control_mode_report import ControlModeReport_Constants
from simulator_compatibility_test.subscribers.control_mode_report import SubscriberControlModeReport


class ControlModeCommand_Constants(Enum):
    NO_COMMAND = 0
    AUTONOMOUS = 1
    MANUAL = 2


class Test01ControlModeAndReportBase:
    msgs_rx = []
    node = None
    sub = None
    pub = None
    sub_control_mode_report = None
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
        cls.node = rclpy.create_node("test_01_control_mode_and_report_base")
        cls.sub = cls.node.create_subscription(
            ControlModeCommand,
            "/control/command/control_mode_cmd",
            lambda msg: cls.msgs_rx.append(msg),
            10,
        )
        cls.pub = cls.node.create_publisher(
            ControlModeCommand, "/control/command/control_mode_cmd", QOS_RKL10TL
        )
        cls.sub_control_mode_report = SubscriberControlModeReport()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_control_mode_report)
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_method(self):
        self.msgs_rx.clear()

    def set_control_mode(self, control_mode):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_control_mode_cmd_msg(control_mode))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert received.mode == control_mode.value
        self.msgs_rx.clear()

    def generate_control_mode_cmd_msg(self, control_mode):
        stamp = self.node.get_clock().now().to_msg()
        msg = ControlModeCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.mode = control_mode.value
        return msg

    def get_control_mode_report(self):
        time.sleep(1)
        self.sub_control_mode_report.received.clear()
        received = ControlModeReport_Constants.NO_COMMAND
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_control_mode_report.received) > 2:
                    break
            received = self.sub_control_mode_report.received.pop()
        finally:
            return received
