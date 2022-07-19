from enum import Enum

from autoware_auto_vehicle_msgs.msg import GearCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class GearMode(Enum):
    NONE = 0
    NEUTRAL = 1
    DRIVE = 2
    REVERSE = 20
    PARK = 22
    LOW = 23


class PublisherGearCommand(Node):
    def __init__(self):
        super().__init__("gear_command_publisher")

        self.declare_parameter("qos_depth", 10)
        qos_depth = self.get_parameter("qos_depth").value

        QOS_RKL10TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.topic = "/control/command/gear_cmd"
        self.publisher_ = self.create_publisher(GearCommand, self.topic, QOS_RKL10TL)

    def publish_msg(self, gear_mode):
        stamp = self.get_clock().now().to_msg()
        msg = GearCommand()
        msg.stamp.sec = stamp.sec
        msg.stamp.nanosec = stamp.nanosec
        msg.command = gear_mode.value
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PublisherGearCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
