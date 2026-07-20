import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import (
    VelocityReport, SteeringReport, ControlModeReport, GearReport)
from autoware_vehicle_msgs.srv import ControlModeCommand

class FakeVehicleStatus(Node):
    def __init__(self):
        super().__init__('fake_vehicle_status')
        self.pv = self.create_publisher(VelocityReport,  '/vehicle/status/velocity_status', 10)
        self.ps = self.create_publisher(SteeringReport,  '/vehicle/status/steering_status', 10)
        self.pm = self.create_publisher(ControlModeReport,'/vehicle/status/control_mode', 10)
        self.pg = self.create_publisher(GearReport,      '/vehicle/status/gear_status', 10)
        # Vehicle interface offers this so Autoware can switch the vehicle into
        # autonomous control when the operator engages; without it the operation
        # mode transition to Autonomous times out.
        self.srv = self.create_service(
            ControlModeCommand, '/control/control_mode_request', self.on_control_mode_request)
        # Report MANUAL until Autoware requests autonomous (mimics a real vehicle).
        self.control_mode = ControlModeReport.MANUAL
        self.create_timer(0.02, self.tick)   # 50 Hz

    def on_control_mode_request(self, request, response):
        # ControlModeCommand and ControlModeReport share the same mode constants.
        self.control_mode = request.mode
        response.success = True
        return response

    def tick(self):
        now = self.get_clock().now().to_msg()
        v = VelocityReport(); v.header.stamp = now; v.header.frame_id = 'base_link'
        v.longitudinal_velocity = 0.0; v.lateral_velocity = 0.0; v.heading_rate = 0.0
        self.pv.publish(v)
        s = SteeringReport(); s.stamp = now; s.steering_tire_angle = 0.0
        self.ps.publish(s)
        m = ControlModeReport(); m.stamp = now; m.mode = self.control_mode
        self.pm.publish(m)
        # DRIVE (not PARK) so planning/engage treats the vehicle as ready to move.
        g = GearReport(); g.stamp = now; g.report = GearReport.DRIVE
        self.pg.publish(g)

def main():
    rclpy.init(); rclpy.spin(FakeVehicleStatus())

if __name__ == '__main__':
    main()
