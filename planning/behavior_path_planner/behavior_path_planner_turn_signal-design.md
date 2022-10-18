# Turn Signal Decider

Turn Signal decider determines necessary blinkers.

## Purpose / Role

This module is responsible for activating a necessary blinker during driving. It uses rule-based algorithm to determine blinkers, and the details of this algorithm are described in the following sections. Note that this algorithm is strictly based on the Japanese Road Traffic Raw.

### Assumptions

Autoware has following order of priorities for turn signals.

1. Activate turn signal to safely navigate ego vehicle and protect other road participants
2. Follow traffic laws
3. Follow human driving practices

### Limitations

Currently, this algorithm can sometimes give unnatural (not wrong) blinkers in a complicated situations. This is because it tries to follow the road traffic raw and cannot solve `blinker conflicts` clearly in that environment.

## Parameters for turn signal decider

| Name                                            | Unit | Type   | Description                                                                  | Default value |
| :---------------------------------------------- | :--- | :----- | :--------------------------------------------------------------------------- | :------------ |
| turn_signal_intersection_search_distance        | [m]  | double | constant search distance to decide activation of blinkers at intersections   | 30            |
| turn_signal_intersection_angle_threshold_degree | deg  | double | angle threshold to determined the end point of intersection required section | 15            |
| turn_signal_minimum_search_distance             | [m]  | double | minimum search distance for avoidance and lane change                        | 10            |
| turn_signal_search_time                         | [s]  | double | search time for to decide activation of blinkers                             | 3.0           |
| turn_signal_shift_length_threshold              | [m]  | double | shift length threshold to decide activation of blinkers                      | 0.3           |

Note that the default values for `turn_signal_intersection_search_distance` and `turn_signal_search_time` is strictly followed by [Japanese Road Traffic Laws](https://www.japaneselawtranslation.go.jp/ja/laws/view/2962). So if your country does not allow to use these default values, you should change these values in configuration files.

## Inner-workings / Algorithms

In this algorithm, it assumes that each blinker has two sections, which are `desired section` and `required section`. The image of these two sections are depicted in the following diagram.

![section_definition](./image/turn_signal_decider/sections_definition.drawio.svg)

These two sections have the following meanings.

### - Desired Section

    - This section is defined by road traffic laws. It cannot be longer or shorter than the designated length defined by the law.
    - In this section, you do not have to activate the designated blinkers if it is dangerous to do so.

### - Required Section

    - In this section, ego vehicle must activate designated blinkers. However, if there are blinker conflicts, it must solve them based on the algorithm we mention later in this document.
    - Required section cannot be longer than desired section.

For left turn, right turn, avoidance, lane change, pull over and pull out, we define these two sections, which are elaborated in the following part.

#### 1. Left and Right turn

Turn signal decider checks each lanelet on the map if it has `turn_direction` information. If a lanelet has this information, it activates necessary blinker based on this information.

- desired start point  
  The `search_distance` for blinkers at intersections is `v * turn_signal_search_time + turn_signal_intersection_search_distance`. Then the start point becomes `search_distance` meters before the start point of the intersection lanelet(depicted in gree in the following picture), where `v` is the velocity of the ego vehicle. However, if we set `turn_signal_distance` in the lanelet, we use that length as search distance.

- desired end point  
  Terminal point of the intersection lanelet.

- required start point  
  Initial point of the intersection lanelet.

- required end point  
  The earliest point that satisfies the following condition. $\theta - \theta_{\textrm{end}} < \delta$, where $\theta_{\textrm{end}}$ is yaw angle of the terminal point of the lanelet, $\theta$ is the angle of a required end point and $\delta$ is the threshold defined by the user.

![section_turn_signal](./image/turn_signal_decider/left_right_turn.drawio.svg)

#### 2. Avoidance

Avoidance can be separated into two sections, first section and second section. The first section is from the start point of the path shift to the end of the path shift. The second section is from the end of shift point to the end of avoidance. Note that avoidance module will not activate turn signal when its shift length is below `turn_signal_shift_length_threshold`.

First section

- desired start point  
  `v * turn_signal_search_time` meters before the start point of the avoidance shift path.

- desired end point  
  Shift complete point where the path shift is completed.

- required start point  
  Avoidance start point.

- required end point  
  Shift complete point same as the desired end point.

![section_first_avoidance](./image/turn_signal_decider/avoidance_first_section.drawio.svg)

Second section

- desired start point  
  Shift complete point.

- desired end point  
  Avoidance terminal point

- required start point  
  Shift complete point.

- required end point  
  Avoidance terminal point.

![section_second_avoidance](./image/turn_signal_decider/avoidance_second_section.drawio.svg)

#### 3. Lane Change

- desired start point  
  `v * turn_signal_search_time` meters before the start point of the lane change path.

- desired end point  
  Terminal point of the lane change path.

- required start point  
  Initial point of the lane change path.

- required end point  
  Cross point with lane change path and boundary line of the adjacent lane.

![section_lane_change](./image/turn_signal_decider/lane_change.drawio.svg)

#### 4. Pull out

- desired start point  
  Start point of the path of pull out.

- desired end point  
  Terminal point of the path of pull out.

- required start point  
  Start point of the path pull out.

- required end point  
  Terminal point of the path of pull out.

![section_pull_out](./image/turn_signal_decider/pull_out.drawio.svg)

#### 5. Pull over

- desired start point  
  `v * turn_signal_search_time` meters before the start point of the pull over path.

- desired end point  
  Terminal point of the path of pull over.

- required start point  
  Start point of the path of pull over.

- required end point  
  Terminal point of the path of pull over.

![section_pull_over](./image/turn_signal_decider/pull_over.drawio.svg)

When it comes to handle several blinkers, it gives priority to the first blinker that comes first. However, this rule sometimes activate unnatural blinkers, so turn signal decider uses the following five rules to decide the necessary turn signal.

- pattern1  
  ![pattern1](./image/turn_signal_decider/pattern1.drawio.svg)

- pattern2  
  ![pattern2](./image/turn_signal_decider/pattern2.drawio.svg)

- pattern3  
  ![pattern3](./image/turn_signal_decider/pattern3.drawio.svg)

- pattern4  
  ![pattern4](./image/turn_signal_decider/pattern4.drawio.svg)

- pattern5  
  ![pattern5](./image/turn_signal_decider/pattern5.drawio.svg)

Based on these five rules, turn signal decider can solve `blinker conflicts`. The following pictures show some examples of this kind of conflicts.

#### - Several right and left turns on short sections

In this scenario, ego vehicle has to pass several turns that are close each other. Since this pattern can be solved by the pattern1 rule, the overall result is depicted in the following picture.

![continuous_turns](./image/turn_signal_decider/continuous_turns.drawio.svg)

#### - Avoidance with left turn (1)

In this scene, ego vehicle has to deal with the obstacle that is on its original path as well as make a left turn. The overall result can be varied by the position of the obstacle, but the image of the result is described in the following picture.

![avoidance_and_turn](./image/turn_signal_decider/avoidance_and_turn.drawio.svg)

#### - Avoidance with left turn (2)

Same as the previous scenario, ego vehicle has to avoid the obstacle as well as make a turn left. However, in this scene, the obstacle is parked after the intersection. Similar to the previous one, the overall result can be varied by the position of the obstacle, but the image of the result is described in the following picture.

![avoidance_and_turn](./image/turn_signal_decider/avoidance_and_turn2.drawio.svg)

#### - Lane change and left turn

In this scenario, ego vehicle has to do lane change before making a left turn. In the following example, ego vehicle does not activate left turn signal until it reaches the end point of the lane change path.

![avoidance_and_turn](./image/turn_signal_decider/lane_change_and_turn.drawio.svg)
