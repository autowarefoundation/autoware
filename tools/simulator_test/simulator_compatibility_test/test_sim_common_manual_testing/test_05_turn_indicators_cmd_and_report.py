from simulator_compatibility_test.subscribers.turn_indicators_report import (
    TurnIndicatorsReport_Constants,
)
from test_base.test_05_turn_indicators_cmd_and_report import Test05TurnIndicatorsCmdAndReportBase
from test_base.test_05_turn_indicators_cmd_and_report import TurnIndicatorsCommand_Constants


class Test05TurnIndicatorsCmdAndReportSim(Test05TurnIndicatorsCmdAndReportBase):
    def test_1_enable_left(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.ENABLE_LEFT
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.ENABLE_LEFT.value

    def test_2_enable_right(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.ENABLE_RIGHT
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.ENABLE_RIGHT.value

    def test_3_disable(self, setup_method):
        turn_indicators = TurnIndicatorsCommand_Constants.DISABLE
        self.set_turn_indicators(turn_indicators)
        result = self.get_turn_indicators_status()
        assert result.report == TurnIndicatorsReport_Constants.DISABLE.value
