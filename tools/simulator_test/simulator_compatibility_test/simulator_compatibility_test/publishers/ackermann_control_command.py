from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_control_msgs.msg import AckermannLateralCommand
from autoware_auto_control_msgs.msg import LongitudinalCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class PublisherAckermannControlCommand(Node):
    def __init__(self):
        super().__init__("ackermann_control_command_publisher")

        self.declare_parameter("qos_depth", 10)
        qos_depth = self.get_parameter("qos_depth").value

        QOS_RKL10TL = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.topic = "/control/command/control_cmd"
        self.publisher_ = self.create_publisher(AckermannControlCommand, self.topic, QOS_RKL10TL)

    def publish_msg(self, control_cmd):
        stamp = self.get_clock().now().to_msg()
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

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = PublisherAckermannControlCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
