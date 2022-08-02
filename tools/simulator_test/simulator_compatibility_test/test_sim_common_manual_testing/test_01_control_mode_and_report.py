import pytest
from simulator_compatibility_test.publishers.control_mode_command import (
    ControlModeCommand_Constants,
)
from simulator_compatibility_test.subscribers.control_mode_report import ControlModeReport_Constants
from test_base.test_01_control_mode_and_report import Test01ControlModeAndReportBase


class Test01ControlModeAndReportSim(Test01ControlModeAndReportBase):
    @pytest.mark.skip(reason="manual mode is not required for now")
    def test_1_manual_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.MANUAL)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.MANUAL.value

    def test_2_auto_mode(self, setup_method):
        self.set_control_mode(ControlModeCommand_Constants.AUTONOMOUS)
        result = self.get_control_mode_report()
        assert result == ControlModeReport_Constants.AUTONOMOUS.value
