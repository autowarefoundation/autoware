import time

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_control_msgs.msg import AckermannLateralCommand
from autoware_auto_control_msgs.msg import LongitudinalCommand
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from simulator_compatibility_test.subscribers.velocity_report import SubscriberVelocityReport


class Test03LongitudinalCommandAndReportBase:
    msgs_rx = []
    control_cmd = {}
    node = None
    sub = None
    pub = None
    sub_velocity_report = None
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
        cls.control_cmd = {
            "lateral": {"steering_tire_angle": 0.0, "steering_tire_rotation_rate": 0.0},
            "longitudinal": {"speed": 0.0, "acceleration": 0.0, "jerk": 0.0},
        }

        cls.node = rclpy.create_node("test_03_longitudinal_command_and_report_base")
        cls.sub = cls.node.create_subscription(
            AckermannControlCommand,
            "/control/command/control_cmd",
            lambda msg: cls.msgs_rx.append(msg),
            10,
        )
        cls.pub = cls.node.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", QOS_RKL10TL
        )
        cls.sub_velocity_report = SubscriberVelocityReport()
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.sub_velocity_report)
        cls.executor.add_node(cls.node)

    @classmethod
    def teardown_class(cls) -> None:
        cls.node.destroy_node()
        cls.executor.shutdown()
        rclpy.shutdown()

    @pytest.fixture
    def setup_and_teardown(self):
        yield time.sleep(3)
        self.control_cmd["longitudinal"]["speed"] = 0.0
        self.control_cmd["longitudinal"]["acceleration"] = 0.0
        self.init_vehicle()
        time.sleep(3)

    def init_vehicle(self):
        self.set_speed(0.0)

    def generate_control_msg(self, control_cmd):
        stamp = self.node.get_clock().now().to_msg()
        msg = AckermannControlCommand()
        lateral_cmd = AckermannLateralCommand()
        longitudinal_cmd = LongitudinalCommand()
        lateral_cmd.stamp.sec = stamp.sec
        lateral_cmd.stamp.nanosec = stamp.nanosec
        lateral_cmd.steering_tire_angle = control_cmd["lateral"]["steering_tire_angle"]
        lateral_cmd.steering_tire_rotation_rate = control_cmd["lateral"][
            "steering_tire_rotation_rate"
        ]
        longitudinal_cmd.stamp.sec = stamp.sec
        longitudinal_cmd.stamp.nanosec = stamp.nanosec
        longitudinal_cmd.speed = control_cmd["longitudinal"]["speed"]
        longitudinal_cmd.acceleration = control_cmd["longitudinal"]["acceleration"]
        longitudinal_cmd.jerk = control_cmd["longitudinal"]["jerk"]

        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.lateral = lateral_cmd
        msg.longitudinal = longitudinal_cmd
        return msg

    def set_speed(self, speed):
        self.control_cmd["longitudinal"]["speed"] = speed
        self.msgs_rx.clear()
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert received.longitudinal.speed == speed
        self.msgs_rx.clear()

    def set_acceleration(self, acceleration):
        self.control_cmd["longitudinal"]["acceleration"] = acceleration
        self.msgs_rx.clear()
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=1)
            self.pub.publish(self.generate_control_msg(self.control_cmd))
            if len(self.msgs_rx) > 2:
                break
        received = self.msgs_rx[-1]
        assert received.longitudinal.acceleration == acceleration
        self.msgs_rx.clear()

    def get_velocity_report(self):
        self.sub_velocity_report.received.clear()
        received = 0.0
        try:
            while rclpy.ok():
                self.executor.spin_once(timeout_sec=1)
                if len(self.sub_velocity_report.received) > 2:
                    break
            received = self.sub_velocity_report.received.pop()
        finally:
            return received
