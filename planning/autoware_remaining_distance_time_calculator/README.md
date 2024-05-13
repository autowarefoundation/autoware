## Remaining Distance and Time Calculator

### Role

This package aims to provide mission remaining distance and remaining time calculations.

### Activation and Timing

- The calculations are activated once we have a route planned for a mission in Autoware.
- The calculations are triggered timely based on the `update_rate` parameter.

### Module Parameters

| Name          | Type   | Default Value | Explanation                 |
| ------------- | ------ | ------------- | --------------------------- |
| `update_rate` | double | 10.0          | Timer callback period. [Hz] |

### Inner-workings

#### Remaining Distance Calculation

- The remaining distance calculation is based on getting the remaining shortest path between the current vehicle pose and goal pose using `lanelet2` routing APIs.
- The remaining distance is calculated by summing the 2D length of remaining shortest path, with exception to current lanelet and goal lanelet.
  - For the current lanelet, the distance is calculated from the current vehicle position to the end of that lanelet.
  - For the goal lanelet, the distance is calculated from the start of the lanelet to the goal pose in this lanelet.
- When there is only one lanelet remaining, the distance is calculated by getting the 2D distance between the current vehicle pose and goal pose.
- Checks are added to handle cases when current lanelet, goal lanelet, or routing graph are not valid to prevent node process die.
  - In such cases when, last valid remaining distance and time are maintained.

#### Remaining Time Calculation

- The remaining time currently depends on a simple equation of motion by getting the maximum velocity limit.
- The remaining distance is calculated by dividing the remaining distance by the maximum velocity limit.
- A check is added to the remaining time calculation to make sure that maximum velocity limit is greater than zero. This prevents division by zero or getting negative time value.

### Future Work

- Find a more efficient way for remaining distance calculation instead of regularly searching the graph for finding the remaining shortest path.
- Engage more sophisticated motion models for more accurate remaining time calculations.
