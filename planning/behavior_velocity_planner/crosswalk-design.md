### CrossWalk

#### Role

Judgement whether a vehicle can go into a crosswalk and plan a velocity of the start/stop.

#### Launch Timing

Launches when there is a crosswalk on the target lane.

#### Module Parameters

| Parameter                                                | Type   | Description                                                              |
| -------------------------------------------------------- | ------ | ------------------------------------------------------------------------ |
| `crosswalk/stop_line_distance`                           | double | [m] make stop line away from crosswalk when no explicit stop line exists |
| `crosswalk/stop_margin`                                  | double | [m] a margin that the vehicle tries to stop before stop_line             |
| `crosswalk/slow_margin`                                  | bool   | [m] a margin that the vehicle tries to slow down before stop_line        |
| `crosswalk/slow_velocity`                                | double | [m] a slow down velocity                                                 |
| `crosswalk/stop_predicted_object_prediction_time_margin` | double | [s] time margin for decision of ego vehicle to stop or not               |
| `walkway/stop_line_distance`                             | double | [m] make stop line away from crosswalk when no explicit stop line exists |
| `walkway/stop_margin`                                    | double | [m] a margin that the vehicle tries to stop before walkway               |
| `walkway/stop_duration_sec`                              | double | [s] time margin for decision of ego vehicle to stop                      |

#### Flowchart

flow chart is almost the same as stop line.
