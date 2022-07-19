from enum import Enum

from autoware_auto_vehicle_msgs.msg import ControlModeReport
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class ControlModeReport_Constants(Enum):
    NO_COMMAND = 0
    AUTONOMOUS = 1
    MANUAL = 2
    DISENGAGED = 3
    NOT_READY = 4


class SubscriberControlModeReport(Node):
    def __init__(self):
        super().__init__("control_mode_report_subscriber")

        self.declare_parameter("qos_depth", 10)
        qos_depth = self.get_parameter("qos_depth").value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.topic = "/vehicle/status/control_mode"
        self.subscription_ = self.create_subscription(
            ControlModeReport, self.topic, self.get_control_mode, QOS_RKL10V
        )
        self.received = []

    def get_control_mode(self, msg):
        self.received.append(msg.mode)


def main(args=None):
    rclpy.init(args=args)

    node = SubscriberControlModeReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
