# tier4_simulated_clock_rviz_plugin

## Purpose

This plugin allows publishing and controlling the simulated ROS time.

## Output

| Name     | Type                        | Description                |
| -------- | --------------------------- | -------------------------- |
| `/clock` | `rosgraph_msgs::msg::Clock` | the current simulated time |

## HowToUse

1. Start rviz and select panels/Add new panel.
   ![select_panel](./images/select_panels.png)
2. Select tier4_clock_rviz_plugin/SimulatedClock and press OK.
   ![select_clock_plugin](./images/select_clock_plugin.png)
3. Use the added panel to control how the simulated clock is published.
   ![use_clock_plugin](./images/use_clock_plugin.png)

   - Pause button: pause/resume the clock.
   - Speed: speed of the clock relative to the system clock.
   - Rate: publishing rate of the clock.
   - Step button: advance the clock by the specified time step.
   - Time step: value used to advance the clock when pressing the step button d).
   - Time unit: time unit associated with the value from e).
