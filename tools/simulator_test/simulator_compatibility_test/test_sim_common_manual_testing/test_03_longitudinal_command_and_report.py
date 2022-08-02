import time

from test_base.test_03_longitudinal_command_and_report import Test03LongitudinalCommandAndReportBase


class Test03LongitudinalCommandAndReportSim(Test03LongitudinalCommandAndReportBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_1_longitudinal_control(self, setup_and_teardown):
        self.set_speed(5.0)
        self.set_acceleration(1.0)
        time.sleep(10)
        current_speed = self.get_velocity_report()
        assert current_speed.longitudinal_velocity > 5.0
