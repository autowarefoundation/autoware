from autoware_auto_vehicle_msgs.msg import VelocityReport
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class SubscriberVelocityReport(Node):
    def __init__(self):
        super().__init__("velocity_report_subscriber")

        self.declare_parameter("qos_depth", 10)
        qos_depth = self.get_parameter("qos_depth").value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.topic = "/vehicle/status/velocity_status"
        self.subscription_ = self.create_subscription(
            VelocityReport, self.topic, self.get_velocity, QOS_RKL10V
        )

        self.received = []

    def get_velocity(self, msg):
        self.received.append(msg)


def main(args=None):
    rclpy.init(args=args)

    node = SubscriberVelocityReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
