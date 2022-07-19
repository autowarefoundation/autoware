from enum import Enum

from autoware_auto_vehicle_msgs.msg import ControlModeCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class ControlModeCommand_Constants(Enum):
    NO_COMMAND = 0
    AUTONOMOUS = 1
    MANUAL = 2


class PublisherControlModeCommand(Node):
    def __init__(self):
        super().__init__("control_mode_command_publisher")

        self.declare_parameter("qos_depth", 10)
        qos_depth = self.get_parameter("qos_depth").value

        QOS_RKL10TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.topic = "/control/command/control_mode_cmd"
        self.publisher_ = self.create_publisher(ControlModeCommand, self.topic, QOS_RKL10TL)

    def publish_msg(self, control_mode):
        stamp = self.get_clock().now().to_msg()
        msg = ControlModeCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.mode = control_mode.value
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PublisherControlModeCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
