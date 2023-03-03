# simulator_compatibility_test

## Purpose

Test procedures (e.g. test codes) to check whether a certain simulator is compatible with Autoware

## Overview of the test codes

File structure

- test_base
- test_sim_common_manual_testing
- test_morai_sim

1. test_base provides shared methods for testing. Other test codes are created based on functions defined here.
2. test_sim_common_manual_testing provides the most basic functions. Any simulator can be tested using codes here. However, to make these codes usable with any simulators, the codes do not include any features for test automation.
3. test_morai_sim is an automated version of test_sim_common_manual_testing for MORAI SIM: Drive. Thus it includes 'MORAI SIM: Drive'-specific codes. Users of the other simulators may create similar version for their simulator of interest.

## Test Procedures for test_sim_common_manual_testing

### Build process before test

```bash
source install/setup.bash
colcon build --packages-select simulator_compatibility_test
cd src/universe/autoware.universe/tools/simulator_test/simulator_compatibility_test/test_sim_common_manual_testing
```

To run each test case manually

### Test Case #1

1. Run your simulator
2. Load a map and an ego vehicle for the test
3. Run the test using the following command

   ```bash
   python -m pytest test_01_control_mode_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle control mode is changed into Manual (If the simulator has a GUI for this one, it should display the ego is in Manual)
   - Ego vehicle control mode is changed into Auto (If the simulator has a GUI for this one, it should display the ego is in Auto)
5. Check if pytest output is passed or failure

### Test Case #2

1. Run your simulator (If the simulator is already running, skip this part)
2. Load a map and an ego vehicle for the test (If a map and an ego are loaded already, skip this part)
3. Run the test using the following command

   ```bash
   python -m pytest test_02_change_gear_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle gear mode is changed into "P" (If the simulator has a GUI for this one, it should display the gear mode is in "P")
   - Ego vehicle gear mode is changed into "N" (If the simulator has a GUI for this one, it should display the gear mode is in "N")
   - Ego vehicle gear mode is changed into "R" (If the simulator has a GUI for this one, it should display the gear mode is in "R")
   - Ego vehicle gear mode is changed into "D" (If the simulator has a GUI for this one, it should display the gear mode is in "D")
5. Check if pytest output is passed or failure

### Test Case #3

1. Run your simulator (If the simulator is already running, skip this part)
2. Load a map and an ego vehicle for the test (If a map and an ego are loaded already, skip this part)
3. Run the test using the following command

   ```bash
   python -m pytest test_03_longitudinal_command_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle longitudinal velocity is greater than 10 kph (If the simulator has a GUI for this one, it should display the longitudinal velocity is greater than 10 kph)
   - Ego vehicle longitudinal velocity is going below 10 kph. This is an ego vehicle initialize process to ensure the following acceleration is made by longitudinal.acceleration value (If the simulator has a GUI for this one, it should display the longitudinal velocity is less than 10 kph)
   - Ego vehicle longitudinal velocity is greater than 10 kph (If the simulator has a GUI for this one, it should display the longitudinal velocity is greater than 10 kph)
   - Ego vehicle longitudinal velocity is going below 10 kph. This is an ego vehicle reset process to tear down this test case.
5. Check if pytest output is passed or failure

### Test Case #4

1. Run your simulator (If the simulator is already running, skip this part)
2. Load a map and an ego vehicle for the test (If a map and an ego are loaded already, skip this part)
3. Run the test using the following command

   ```bash
   python -m pytest test_04_lateral_command_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle steering and/or tire value is greater than 0 degree (If the simulator has a GUI for this one, it should display the steering and/or tire is greater than 0 degree)
   - Ego vehicle steering and/or tire value is 0 degree. This is a reset process. (If the simulator has a GUI for this one, it should display the steering and/or tire is 0 degree)
   - Ego vehicle steering and/or tire value is less than 0 degree (If the simulator has a GUI for this one, it should display the steering and/or tire is less than 0 degree)
   - Ego vehicle steering and/or tire value is 0 degree. This is a reset process. (If the simulator has a GUI for this one, it should display the steering and/or tire is 0 degree)
5. Check if pytest output is passed or failure

### Test Case #5

1. Run your simulator (If the simulator is already running, skip this part)
2. Load a map and an ego vehicle for the test (If a map and an ego are loaded already, skip this part)
3. Run the test using the following command

   ```bash
   python -m pytest test_05_turn_indicators_cmd_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle left turn indicator is turned on (If the simulator has a GUI for this one, it should display the left turn indicator is turned on)
   - Ego vehicle right turn indicator is turned on (If the simulator has a GUI for this one, it should display the right turn indicator is turned on)
   - Ego vehicle both turn indicators are turned off. This is a reset process. (If the simulator has a GUI for this one, it should display both left and right turn indicators are turned off)
5. Check if pytest output is passed or failure

### Test Case #6

1. Run your simulator (If the simulator is already running, skip this part)
2. Load a map and an ego vehicle for the test (If a map and an ego are loaded already, skip this part)
3. Run the test using the following command

   ```bash
   python -m pytest test_06_hazard_lights_cmd_and_report.py
   ```

4. Check if expected behavior is created within the simulator
   - Ego vehicle hazard lights are turned on (If the simulator has a GUI for this one, it should display the hazard lights are turned on or blinking)
   - Ego vehicle hazard lights are turned off. This is a reset process. (If the simulator has a GUI for this one, it should display the hazard lights are turned off)
5. Check if pytest output is passed or failure

## Test Procedures for test_morai_sim

### Build process before test

```bash
source install/setup.bash
colcon build --packages-select simulator_compatibility_test
cd src/universe/autoware.universe/tools/simulator_test/simulator_compatibility_test/test_morai_sim
```

Detailed process

(WIP)

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                                     | Type                                                    | Description        |
| ---------------------------------------- | ------------------------------------------------------- | ------------------ |
| `/vehicle/status/control_mode`           | `autoware_auto_vehicle_msgs::msg::ControlModeReport`    | for [Test Case #1] |
| `/vehicle/status/gear_status`            | `autoware_auto_vehicle_msgs::msg::GearReport`           | for [Test Case #2] |
| `/vehicle/status/velocity_status`        | `autoware_auto_vehicle_msgs::msg::VelocityReport`       | for [Test Case #3] |
| `/vehicle/status/steering_status`        | `autoware_auto_vehicle_msgs::msg::SteeringReport`       | for [Test Case #4] |
| `/vehicle/status/turn_indicators_status` | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport` | for [Test Case #5] |
| `/vehicle/status/hazard_lights_status`   | `autoware_auto_vehicle_msgs::msg::HazardLightsReport`   | for [Test Case #6] |

### Output

| Name                                   | Type                                                 | Description            |
| -------------------------------------- | ---------------------------------------------------- | ---------------------- |
| `/control/command/control_mode_cmd`    | `autoware_auto_vehicle_msgs/ControlModeCommand`      | for [Test Case #1]     |
| `/control/command/gear_cmd`            | `autoware_auto_vehicle_msgs/GearCommand`             | for [Test Case #2]     |
| `/control/command/control_cmd`         | `autoware_auto_vehicle_msgs/AckermannControlCommand` | for [Test Case #3, #4] |
| `/vehicle/status/steering_status`      | `autoware_auto_vehicle_msgs/TurnIndicatorsCommand`   | for [Test Case #5]     |
| `/control/command/turn_indicators_cmd` | `autoware_auto_vehicle_msgs/HazardLightsCommand`     | for [Test Case #6]     |

## Parameters

None.

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

None.
