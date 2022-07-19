from math import pi
import time

from test_base.test_04_lateral_command_and_report import Test04LateralCommandAndReportBase


class Test04LateralCommandAndReportMorai(Test04LateralCommandAndReportBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_1_grater_than_zero(self, setup_and_teardown):
        target_value = pi / 6
        self.set_steering_tire_angle(target_value)
        time.sleep(3)
        current_report = self.get_steering_report()
        assert current_report.steering_tire_angle > 0.0

    def test_2_less_than_zero(self, setup_and_teardown):
        target_value = (-1) * pi / 6
        self.set_steering_tire_angle(target_value)
        time.sleep(3)
        current_report = self.get_steering_report()
        assert current_report.steering_tire_angle < 0.0
