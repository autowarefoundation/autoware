from test_base.test_02_change_gear_and_report import GearMode
from test_base.test_02_change_gear_and_report import Test02ChangeGearAndReportBase


class Test02ChangeGearAndReportSim(Test02ChangeGearAndReportBase):
    @classmethod
    def setup_class(cls) -> None:
        super().setup_class()

    def test_1_gear_park(self):
        self.set_gear_mode(GearMode.PARK)
        result = self.get_gear_mode()
        assert result == GearMode.PARK.value

    def test_2_gear_neutral(self):
        self.set_gear_mode(GearMode.NEUTRAL)
        result = self.get_gear_mode()
        assert result == GearMode.NEUTRAL.value

    def test_3_gear_reverse(self):
        self.set_gear_mode(GearMode.REVERSE)
        result = self.get_gear_mode()
        assert result == GearMode.REVERSE.value

    def test_4_gear_drive(self):
        self.set_gear_mode(GearMode.DRIVE)
        result = self.get_gear_mode()
        assert result == GearMode.DRIVE.value
