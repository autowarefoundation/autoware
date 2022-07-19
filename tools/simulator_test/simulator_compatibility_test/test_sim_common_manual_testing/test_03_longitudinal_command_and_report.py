import time

from test_base.test_03_longitudinal_command_and_report import Test03LongitudinalCommandAndReportBase


class Test03LongitudinalCommandAndReportSim(Test03LongitudinalCommandAndReportBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_1_speed_control(self, setup_and_teardown):
        target_value = 100.0
        self.set_speed(target_value)
        time.sleep(3)
        current_speed = self.get_velocity_report()
        assert current_speed.longitudinal_velocity > 10.0

    def test_2_acceleration_control(self, setup_and_teardown):
        target_value = 100.0
        self.set_acceleration(target_value)
        time.sleep(3)
        current_speed = self.get_velocity_report()
        assert current_speed.longitudinal_velocity > 10.0
