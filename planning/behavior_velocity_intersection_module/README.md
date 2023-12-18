# Intersection

## Role

The intersection module is responsible for safely passing urban intersections by:

1. checking collisions with upcoming vehicles
2. recognizing the occluded area in the intersection
3. reacting to each color/shape of associated traffic lights

This module is designed to be agnostic to left-hand/right-hand traffic rules and work for crossroads, T-shape junctions, etc. Roundabout is not formally supported in this module.

![topology](./docs/intersection-topology.drawio.svg)

## Activation condition

This module is activated when the path contains the lanes with turn_direction tag. More precisely, if the lane_ids of the path contain the ids of those lanes, corresponding instances of intersection module are activated on each lane respectively.

## Requirements/Limitations

- The HDMap needs to have the information of turn_direction tag (which should be one of straight, left, right) for all the lanes in intersections and right_of_way tag for specific lanes (refer to [RightOfWay](#how-towhy-set-rightofway-tag) section for more details). See [lanelet2_extension document](https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md) for more detail.
- WIP(perception requirements/limitations)
- WIP(sensor visibility requirements/limitations)

## Attention area

The attention area in the intersection is defined as the set of lanes that are conflicting with ego path and their preceding lanes up to `common.attention_area_length` meters. By default RightOfWay tag is not set, so the attention area covers all the conflicting lanes and its preceding lanes as shown in the first row. RightOfWay tag is used to rule out the lanes that each lane has priority given the traffic light relation and turn_direction priority. In the second row, purple lanes are set as the yield_lane of the ego_lane in the RightOfWay tag.

![attention_area](./docs/intersection-attention.drawio.svg)

intersection_area, which is supposed to be defined on the HDMap, is an area converting the entire intersection.

### In-phase/Anti-phase signal group

The terms "in-phase signal group" and "anti-phase signal group" are introduced to distinguish the lanes by the timing of traffic light regulation as shown in below figure.

![phase signal group](./docs/signal-phase-group.drawio.svg)

The set of intersection lanes whose color is in sync with lane L1 is called the in-phase signal group of L1, and the set of remaining lanes is called the anti-phase signal group.

### How-to/Why set RightOfWay tag

Ideally RightOfWay tag is unnecessary if ego has perfect knowledge of all traffic signal information because:

- it can distinguish which conflicting lanes should be checked because they are GREEN currently and possible collision occur with the vehicles on those lanes
- it can distinguish which conflicting lanes can be ignored because they are RED currently and there is no chance of collision with the vehicles on those lanes unless they violate the traffic rule

That allows ego to generate the attention area dynamically using the real time traffic signal information. However this ideal condition rarely holds unless the traffic signal information is provided through the infrastructure. Also there maybe be very complicated/bad intersection maps where multiple lanes overlap in a complex manner.

- If there is an perfect access to entire traffic light signal, then you can set `common.use_map_right_of_way` to false and there is no need to set RightOfWay tag on the map. The intersection module will generate the attention area by checking traffic signal and corresponding conflicting lanes. This feature is not implemented yet.
- If traffic signal information is not perfect, then set `common.use_map_right_of_way` to true. If you do not want to detect vehicles on the anti-phase signal group lanes, set them as yield_lane for ego lane.
- Even if there are no traffic lights if the intersection lanes are overlapped in a ugly manner, you may need to set RightOfWay tag. For example if adjacent intersection lanes of the same in-phase group are not sharing the boundary line and overlapped a little bit, you may need to set RightOfWay to each other for them in order to avoid unnecessary stop for vehicle on such unrelated lane.

To help the intersection module care only a set of limited lanes, RightOfWay tag needs to be properly set.

Following table shows an **example** of how to set yield_lanes to each lane in a intersection w/o traffic lights. Since it is not apparent how to uniquely determine signal phase group for a set of intersection lanes in geometric/topological manner, yield_lane needs to be set manually. Straight lanes with traffic lights are exceptionally handled to detect no lanes because commonly it has priority over all the other lanes, so no RightOfWay setting is required.

| turn direction of right_of_way | yield_lane(with traffic light)                                                              | yield_lane(without traffic light)              |
| ------------------------------ | ------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| straight                       | not need to set yield_lane(this case is special)                                            | left/right conflicting lanes of in-phase group |
| left(Left hand traffic)        | all conflicting lanes of the anti-phase group and right conflicting lanes of in-phase group | right conflicting lanes of in-phase group      |
| right(Left hand traffic)       | all conflicting lanes of the anti-phase group                                               | no yield_lane                                  |
| left(Right hand traffic)       | all conflicting lanes of the anti-phase group                                               | no yield_lane                                  |
| right(Right hand traffic)      | all conflicting lanes of the anti-phase group and right conflicting lanes of in-phase group | left conflicting lanes of in-phase group       |

This setting gives the following `attention_area` configurations.

![attention_area_straight](./docs/intersection-attention-straight.drawio.svg)
![attention_area_ll_rr](./docs/intersection-attention-ll-rr.drawio.svg)
![attention_area_lr_rl](./docs/intersection-attention-lr-rl.drawio.svg)

For complex/bad intersection map like the one illustrated below, additional RightOfWay setting maybe necessary.

![bad-map](./docs/ugly-intersection.drawio.svg)

The bad points are:

1. ego lane is overlapped with adjacent lane of the in-phase group. In this case you need to set this lane as yield_lane additionally because otherwise attention area is generated for its preceding lanes as well, which may cause unwanted stop.
2. ego lane is overlapped with unrelated lane. In this case the lane is right-turn only and there is no chance of collision in theory. But you need to set this lane as yield_lane additionally for the same reason as (1).

## Possible stop lines

Following figure illustrates important positions used in the intersection module. Note that each solid line represents ego front line position and the corresponding dot represents the actual inserted stop point position for the vehicle frame, namely the center of the rear wheel.

![data structure](./docs/intersection-stoplines.drawio.svg)

To precisely calculate stop positions, the path is interpolated at the certain interval of `common.path_interpolation_ds`.

- closest_idx denotes the path point index which is closest to ego position.
- first_attention_stopline denotes the first path point where ego footprint intersects with the attention_area.
- If a stopline is associated with the intersection lane on the map, that line is used as the default_stopline for collision detection. Otherwise the point which is `common.default_stopline_margin` meters behind first_attention_stopline is defined as the default_stopline instead.
- occlusion_peeking_stopline is a bit ahead of first_attention_stopline as described later.
- occlusion_wo_tl_pass_judge_line is the first position where ego footprint intersects with the centerline of the first attention_area lane.

## Target objects

For [stuck vehicle detection](#stuck-vehicle-detection) and [collision detection](#collision-detection), this module checks **car**, **bus**, **truck**, **trailer**, **motor cycle**, and **bicycle** type objects.

Objects that satisfy all of the following conditions are considered as target objects (possible collision objects):

- The center of the object is **within a certain distance** from the attention lane (threshold = `common.attention_area_margin`) .
  - (Optional condition) The center of the object is in the **intersection area**.
    - To deal with objects that is in the area not covered by the lanelets in the intersection.
- The posture of object is **the same direction as the attention lane** (threshold = `common.attention_area_angle_threshold`).
- Not being **in the adjacent lanes of ego**.

## Overview of decision process

There are several behaviors depending on the scene.

| behavior                 | scene                                                                                             | action                                                                              |
| ------------------------ | ------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------- |
| Safe                     | Ego detected no occlusion and collision                                                           | Ego passes the intersection                                                         |
| StuckStop                | The exit of the intersection is blocked by traffic jam                                            | Ego stops before the intersection or the boundary of attention area                 |
| YieldStuck               | Another vehicle stops to yield ego                                                                | Ego stops before the intersection or the boundary of attention area                 |
| NonOccludedCollisionStop | Ego detects no occlusion but detects collision                                                    | Ego stops at the default_stop_line                                                  |
| FirstWaitBeforeOcclusion | Ego detected occlusion when entering the intersection                                             | Ego stops at the default_stop_line at first                                         |
| PeekingTowardOcclusion   | Ego detected occlusion and but no collision within the FOV (after FirstWaitBeforeOcclusion)       | Ego approaches the boundary of the attention area slowly                            |
| OccludedCollisionStop    | Ego detected both occlusion and collision (after FirstWaitBeforeOcclusion)                        | Ego stops immediately                                                               |
| FullyPrioritized         | Ego is fully prioritized by the RED/Arrow signal                                                  | Ego only cares vehicles still running inside the intersection. Occlusion is ignored |
| OverPassJudgeLine        | Ego is already inside the attention area and/or cannot stop before the boundary of attention area | Ego does not detect collision/occlusion anymore and passes the intersection         |

```plantuml
@startuml

state begin <<choice>>
[*] --> begin
begin --> OverPassJudgeLine: IF over_pass_judge

state "Before pass judge line" as NotOverPassJudgeLine {
state check_stuck <<choice>>
begin --> check_stuck: ELSE

check_stuck --> StuckStop: IF stuck vehicle detected

state check_yield_stuck <<choice>>
check_stuck --> check_yield_stuck: ELSE
check_yield_stuck --> YieldStuck: IF yield stuck vehicle detected

state check_tl_priority <<choice>>
check_yield_stuck --> check_tl_priority: ELSE

state check_occlusion <<choice>>
check_tl_priority --> check_occlusion: IF not prioritized

state Safe
State "Prioritized by traffic light" as Prioritized {
state check_collision_prioritized <<choice>>
check_tl_priority --> check_collision_prioritized: IF prioritized
State FullyPrioritized
check_collision_prioritized --> FullyPrioritized: IF collision detected
check_collision_prioritized --> Safe: ELSE
}

check_occlusion --> Occlusion: IF occlusion is detected

State "Occlusion is not detected" as NoOcclusion {
state check_collision <<choice>>
check_occlusion --> check_collision: ELSE
State NonOccludedCollisionStop
check_collision --> Safe: ELSE
check_collision --> NonOccludedCollisionStop: IF collision detected
}

State "Occlusion is detected" as Occlusion {
state FirstWaitBeforeOcclusion
FirstWaitBeforeOcclusion --> Peeking: after termporal stop
state Peeking {
State PeekingTowardOcclusion
State OccludedCollisionStop
PeekingTowardOcclusion --> OccludedCollisionStop : IF collision detected
OccludedCollisionStop --> PeekingTowardOcclusion: IF not collision detected
}
}

}

@enduml
```

## Stuck Vehicle Detection

If there is any object on the path inside the intersection and at the exit of the intersection (up to `stuck_vehicle.stuck_vehicle_detect_dist`) lane and its velocity is less than the threshold (`stuck_vehicle.stuck_vehicle_velocity_threshold`), the object is regarded as a stuck vehicle. If stuck vehicles exist, this module inserts a stopline a certain distance (=`default_stopline_margin`) before the overlapped region with other lanes. The stuck vehicle detection area is generated based on the planned path, so the stuck vehicle stopline is not inserted if the upstream module generated an avoidance path.

![stuck_vehicle_detection](./docs/stuck-vehicle.drawio.svg)

## Yield stuck vehicle detection

If there is any stopped object on the attention lanelet between the intersection point with ego path and the position which is `yield_stuck.distance_threshold` before that position, the object is regarded as yielding to ego vehicle. In this case ego is given the right-of-way by the yielding object but this module inserts stopline to prevent entry into the intersection. This scene happens when the object is yielding against ego or the object is waiting before the crosswalk around the exit of the intersection.

![yield_stuck_detection](./docs/yield-stuck.drawio.svg)

## Collision detection

The following process is performed for the targets objects to determine whether ego can pass the intersection safely. If it is judged that ego cannot pass the intersection with enough margin, this module inserts a stopline on the path.

1. predict the time $t$ when the object intersects with ego path for the first time from the predicted path time step. Only the predicted whose confidence is greater than `collision_detection.min_predicted_path_confidence` is used.
2. detect collision between the predicted path and ego's predicted path in the following process
   1. calculate the collision interval of [$t$ - `collision_detection.collision_start_margin_time`, $t$ + `collision_detection.collision_end_margin_time`]
   2. calculate the passing area of ego during the collision interval from the array of (time, distance) obtained by smoothed velocity profile
   3. check if ego passing area and object predicted path interval collides
3. if collision is detected, the module inserts a stopline
4. if ego is over the [pass_judge_line](#pass-judge-line), collision checking is skipped to avoid sudden braking and/or unnecessary stop in the middle of the intersection

The parameters `collision_detection.collision_start_margin_time` and `collision_detection.collision_end_margin_time` can be interpreted as follows:

- If ego was to pass the intersection earlier than the target object, collision would be detected if the time difference between the two was less than `collision_detection.collision_start_margin_time`.
- If ego was to pass the intersection later than the target object, collision would be detected if the time difference between the two was less than `collision_detection.collision_end_margin_time`.

If collision is detected, the state transits to "STOP" immediately. On the other hand, the state does not transit to "GO" unless safe judgement continues for a certain period `collision_detection.collision_detection_hold_time` to prevent the chattering of decisions.

Currently, the intersection module uses `motion_velocity_smoother` feature to precisely calculate ego velocity profile along the intersection lane under longitudinal/lateral constraints. If the flag `collision_detection.velocity_profile.use_upstream` is true, the target velocity profile of the original path is used. Otherwise the target velocity is set to `collision.velocity_profile.default_velocity`. In the trajectory smoothing process the target velocity at/before ego trajectory points are set to ego current velocity. The smoothed trajectory is then converted to an array of (time, distance) which indicates the arrival time to each trajectory point on the path from current ego position. You can visualize this array by adding the lane id to `debug.ttc` and running

```bash
ros2 run behavior_velocity_intersection_module ttc.py --lane_id <lane_id>
```

![ego ttc profile](./docs/ttc.gif)

## Occlusion detection

If the flag `occlusion.enable` is true this module checks if there is sufficient field of view (FOV) on the attention area up to `occlusion.occlusion_attention_area_length`. If FOV is not clear enough ego first makes a brief stop at the default stop line for `occlusion.temporal_stop_time_before_peeking`, and then slowly creeps toward occlusion_peeking_stop_line. If `occlusion.creep_during_peeking.enable` is true `occlusion.creep_during_peeking.creep_velocity` is inserted up to occlusion_peeking_stop_line. Otherwise only stop line is inserted.

During the creeping if collision is detected this module inserts a stop line in front of ego immediately, and if the FOV gets sufficiently clear the intersection_occlusion wall will disappear. If occlusion is cleared and no collision is detected ego will pass the intersection.

The occlusion is detected as the common area of occlusion attention area(which is partially the same as the normal attention area) and the unknown cells of the occupancy grid map. The occupancy grid map is denoised using morphology with the window size of `occlusion.denoise_kernel`. The occlusion attention area lanes are discretized to line strings and they are used to generate a grid whose each cell represents the distance from ego path along the lane as shown below.

![occlusion_detection](./docs/occlusion_grid.drawio.svg)

If the nearest occlusion cell value is below the threshold `occlusion.occlusion_required_clearance_distance`, it means that the FOV of ego is not clear. It is expected that the occlusion gets cleared as the vehicle approaches the occlusion peeking stop line.

### Occlusion source estimation at intersection with traffic light

At intersection with traffic light, the whereabout of occlusion is estimated by checking if there are any objects between ego and the nearest occlusion cell. While the occlusion is estimated to be caused by some object (DYNAMICALLY occluded), intersection_wall appears at all times. If no objects are found between ego and the nearest occlusion cell (STATICALLY occluded), after ego stopped for the duration of `occlusion.static_occlusion_with_traffic_light_timeout` plus `occlusion.occlusion_detection_hold_time`, occlusion is intentionally ignored to avoid stuck.

![occlusion_detection](./docs/occlusion-with-tl.drawio.svg)

The remaining time is visualized on the intersection_occlusion virtual wall.

![static-occlusion-timeout](./docs/static-occlusion-timeout.png)

### Occlusion handling at intersection without traffic light

At intersection without traffic light, if occlusion is detected, ego makes a brief stop at the default_stopline and first_attention_stopline respectively. After stopping at the first_attention_area_stopline this module inserts `occlusion.absence_traffic_light.creep_velocity` velocity between ego and occlusion_wo_tl_pass_judge_line while occlusion is not cleared. If collision is detected, ego immediately stops. Once the occlusion is cleared or ego has passed occlusion_wo_tl_pass_judge_line this module does not detect collision and occlusion because ego footprint is already inside the intersection.

![occlusion_detection](./docs/occlusion-without-tl.drawio.svg)

While ego is creeping, yellow intersection_wall appears in front ego.

![occlusion-wo-tl-creeping](./docs/occlusion-wo-tl-creeping.png)

## Traffic signal specific behavior

### Collision detection

TTC parameter varies depending on the traffic light color/shape as follows.

| traffic light color | ttc(start)                                                             | ttc(end)                                                               |
| ------------------- | ---------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| GREEN               | `collision_detection.not_prioritized.collision_start_margin`           | `collision_detection.not_prioritized.collision_end_margin`             |
| AMBER               | `collision_detection.partially_prioritized.collision_start_end_margin` | `collision_detection.partially_prioritized.collision_start_end_margin` |
| RED / Arrow         | `collision_detection.fully_prioritized.collision_start_end_margin`     | `collision_detection.fully_prioritized.collision_start_end_margin`     |

### yield on GREEN

If the traffic light color changed to GREEN and ego approached the entry of the intersection lane within the distance `collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start` and there is any object whose distance to its stopline is less than `collision_detection.yield_on_green_traffic_light.object_dist_to_stopline`, this module commands to stop for the duration of `collision_detection.yield_on_green_traffic_light.duration` at the default_stopline.

### skip on AMBER

If the traffic light color is AMBER but the object is expected to stop before its stopline under the deceleration of `collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration`, collision checking is skipped.

### skip on RED

If the traffic light color is RED or Arrow signal is turned on, the attention lanes which are not conflicting with ego lane are not used for detection. And even if the object stops with a certain overshoot from its stopline, but its expected stop position under the deceleration of `collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration` is more than the distance `collision_detection.ignore_on_red_traffic_light.object_margin_to_path` from collision point, the object is ignored.

### Occlusion detection

When the traffic light color/shape is RED/Arrow, occlusion detection is skipped.

![traffic-light-specific-behavior](./docs/traffic-light-specific-behavior.drawio.svg)

## Pass Judge Line

To avoid sudden braking, if deceleration and jerk more than the threshold (`common.max_accel` and `common.max_jerk`) is required to stop at first_attention_stopline, this module does not command to stop once it passed the default_stopline position.

If ego passed pass_judge_line, then ego does not stop anymore. If ego passed pass_judge_line while ego is stopping for dangerous decision, then ego stops while the situation is judged as dangerous. Once the judgement turned safe, ego restarts and does not stop anymore.

The position of the pass judge line depends on the occlusion detection configuration and the existence of the associated traffic light of the intersection lane.

- If `occlusion.enable` is false, the pass judge line before the `first_attention_stopline` by the braking distance $v_{ego}^{2} / 2a_{max}$.
- If `occlusion.enable` is true and:
  - if there are associated traffic lights, the pass judge line is at the `occlusion_peeking_stopline` in order to continue peeking/collision detection while occlusion is detected.
  - if there are no associated traffic lights and:
    - if occlusion is detected, pass judge line is at the `occlusion_wo_tl_pass_judge_line` to continue peeking.
    - if occlusion is not detected, pass judge line is at the same place at the case where `occlusion.enable` is false.

## Data Structure

Each data structure is defined in `util_type.hpp`.

![data-structure](./docs/data-structure.drawio.svg)

### `IntersectionLanelets`

```plantuml
@startuml
entity IntersectionLanelets {
 * conflicting lanes/area
 --
 * first conflicting area
 The conflicting lane area which the path intersects first
 --
 * attention lanes/area
 --
 * first attention lane area
 The attention lane area which the path intersects first
 --
 * occlusion attention lanes/area
 Part of attention lanes/area for occlusion detection
 --
 * is_priortized: bool
 If ego has priority in current traffic light context
}
@enduml
```

### `IntersectionStopLines`

Each stop lines are generated from interpolated path points to obtain precise positions.

```plantuml
@startuml
entity IntersectionStopLines {
  * closest_idx: size_t
  closest path point index for ego
  --
  * stuck_stopline: size_t
  stop line index on stuck vehicle detection
  --
  * default_stopline: size_t
  If defined on the map, its index on the path. Otherwise generated before first_attention_stopline
  --
  * first_attention_stopline
  The index of the first path point which is inside the attention area
  --
  * occlusion_peeking_stopline
  The index of the path point for the peeking limit position
  --
  * pass_judge_line
  The index of the path point which is before first_attention_stopline/occlusion_peeking_stopline by braking distance
}
@enduml
```

### `TargetObject`

`TargetObject` holds the object, its belonging lane and corresponding stopline information.

```plantuml
@startuml
entity TargetObject {
  * object: PredictedObject
  detected object
  --
  * attention_lanelet: ConstLanelet
  belonging lanelet instance
  --
  * stopline: ConstLineString3d
  reachable stopline of attention_lanelet
}
@enduml
```

## Module Parameters

### common

| Parameter                         | Type   | Description                                                              |
| --------------------------------- | ------ | ------------------------------------------------------------------------ |
| `.attention_area_length`          | double | [m] range for object detection                                           |
| `.attention_area_margin`          | double | [m] margin for expanding attention area width                            |
| `.attention_area_angle_threshold` | double | [rad] threshold of angle difference between the detected object and lane |
| `.use_intersection_area`          | bool   | [-] flag to use intersection_area for collision detection                |
| `.default_stopline_margin`        | double | [m] margin before_stop_line                                              |
| `.stopline_overshoot_margin`      | double | [m] margin for the overshoot from stopline                               |
| `.max_accel`                      | double | [m/ss] max acceleration for stop                                         |
| `.max_jerk`                       | double | [m/sss] max jerk for stop                                                |
| `.delay_response_time`            | double | [s] action delay before stop                                             |

### stuck_vehicle/yield_stuck

| Parameter                                        | Type   | Description                                                                  |
| ------------------------------------------------ | ------ | ---------------------------------------------------------------------------- |
| `stuck_vehicle.turn_direction`                   | -      | [-] turn_direction specifier for stuck vehicle detection                     |
| `stuck_vehicle.stuck_vehicle_detect_dist`        | double | [m] length toward from the exit of intersection for stuck vehicle detection  |
| `stuck_vehicle.stuck_vehicle_velocity_threshold` | double | [m/s] velocity threshold for stuck vehicle detection                         |
| `yield_stuck.distance_threshold`                 | double | [m/s] distance threshold of yield stuck vehicle from ego path along the lane |

### collision_detection

| Parameter                                     | Type   | Description                                                                          |
| --------------------------------------------- | ------ | ------------------------------------------------------------------------------------ |
| `.consider_wrong_direction_vehicle`           | bool   | [-] flag to detect objects in the wrong direction                                    |
| `.collision_detection_hold_time`              | double | [s] hold time of collision detection                                                 |
| `.min_predicted_path_confidence`              | double | [-] minimum confidence value of predicted path to use for collision detection        |
| `.keep_detection_velocity_threshold`          | double | [s] ego velocity threshold for continuing collision detection before pass judge line |
| `.velocity_profile.use_upstream`              | bool   | [-] flag to use velocity profile planned by upstream modules                         |
| `.velocity_profile.minimum_upstream_velocity` | double | [m/s] minimum velocity of upstream velocity profile to avoid zero division           |
| `.velocity_profile.default_velocity`          | double | [m/s] constant velocity profile when use_upstream is false                           |
| `.velocity_profile.minimum_default_velocity`  | double | [m/s] minimum velocity of default velocity profile to avoid zero division            |
| `.yield_on_green_traffic_light`               | -      | [-] [description](#yield-on-green)                                                   |
| `.ignore_amber_traffic_light`                 | -      | [-] [description](#skip-on-amber)                                                    |
| `.ignore_on_red_traffic_light`                | -      | [-] [description](#skip-on-red)                                                      |

### occlusion

| Parameter                                      | Type     | Description                                                                                 |
| ---------------------------------------------- | -------- | ------------------------------------------------------------------------------------------- |
| `.enable`                                      | bool     | [-] flag to calculate occlusion detection                                                   |
| `.occlusion_attention_area_length`             | double   | [m] the length of attention are for occlusion detection                                     |
| `.free_space_max`                              | int      | [-] maximum value of occupancy grid cell to treat at occluded                               |
| `.occupied_min`                                | int      | [-] minimum value of occupancy grid cell to treat at occluded                               |
| `.denoise_kernel`                              | double   | [m] morphology window size for preprocessing raw occupancy grid                             |
| `.attention_lane_crop_curvature_threshold`     | double   | [m] curvature threshold for trimming curved part of the lane                                |
| `.attention_lane_crop_curvature_ds`            | double   | [m] discretization interval of centerline for lane curvature calculation                    |
| `.creep_during_peeking.enable`                 | bool     | [-] flag to insert `creep_velocity` while peeking to intersection occlusion stopline        |
| `.creep_during_peeking.creep_velocity`         | double   | [m/s] the command velocity while peeking to intersection occlusion stopline                 |
| `.peeking_offset`                              | double   | [m] the offset of the front of the vehicle into the attention area for peeking to occlusion |
| `.occlusion_required_clearance_distance`       | double   | [m] threshold for the distance to nearest occlusion cell from ego path                      |
| `.possible_object_bbox`                        | [double] | [m] minimum bounding box size for checking if occlusion polygon is small enough             |
| `.ignore_parked_vehicle_speed_threshold`       | double   | [m/s] velocity threshold for checking parked vehicle                                        |
| `.occlusion_detection_hold_time`               | double   | [s] hold time of occlusion detection                                                        |
| `.temporal_stop_time_before_peeking`           | double   | [s] temporal stop duration at the default_stop_line before starting peeking                 |
| `.temporal_stop_before_attention_area`         | bool     | [-] flag to temporarily stop at first_attention_stopline before peeking into attention_area |
| `.creep_velocity_without_traffic_light`        | double   | [m/s] creep velocity to occlusion_wo_tl_pass_judge_line                                     |
| `.static_occlusion_with_traffic_light_timeout` | double   | [s] the timeout duration for ignoring static occlusion at intersection with traffic light   |

## Trouble shooting

### Intersection module stops against unrelated vehicles

In this case, first visualize `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection` topic and check the `attention_area` polygon. Intersection module performs collision checking for vehicles running on this polygon, so if it extends to unintended lanes, it needs to have [RightOfWay tag](#how-towhy-set-rightofway-tag).

By lowering `common.attention_area_length` you can check which lanes are conflicting with the intersection lane. Then set part of the conflicting lanes as the yield_lane.

### The stop line of intersection is chattering

The parameter `collision_detection.collision_detection_hold_time` suppresses the chattering by keeping UNSAFE decision for this duration until SAFE decision is finally made. The role of this parameter is to account for unstable detection/tracking of objects. By increasing this value you can suppress the chattering. However it could elongate the stopping duration excessively.

If the chattering arises from the acceleration/deceleration of target vehicles, increase `collision_detection.collision_detection.collision_end_margin_time` and/or `collision_detection.collision_detection.collision_end_margin_time`.

### The stop line is released too fast/slow

If the intersection wall appears too fast, or ego tends to stop too conservatively for upcoming vehicles, lower the parameter `collision_detection.collision_detection.collision_start_margin_time`. If it lasts too long after the target vehicle passed, then lower the parameter `collision_detection.collision_detection.collision_end_margin_time`.

### Ego suddenly stops at intersection with traffic light

If the traffic light color changed from AMBER/RED to UNKNOWN, the intersection module works in the GREEN color mode. So collision and occlusion are likely to be detected again.

### Occlusion is detected overly

You can check which areas are detected as occlusion by visualizing `/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection/occlusion_polygons`.

If you do not want to detect / do want to ignore occlusion far from ego or lower the computational cost of occlusion detection, `occlusion.occlusion_attention_area_length` should be set to lower value.

If you want to care the occlusion nearby ego more cautiously, set `occlusion.occlusion_required_clearance_distance` to a larger value. Then ego will approach the occlusion_peeking_stopline more closely to assure more clear FOV.

`occlusion.possible_object_bbox` is used for checking if detected occlusion area is small enough that no vehicles larger than this size can exist inside. By decreasing this size ego will ignore small occluded area.

#### occupancy grid map tuning

Refer to the document of [probabilistic_occupancy_grid_map](https://autowarefoundation.github.io/autoware.universe/main/perception/probabilistic_occupancy_grid_map/) for details. If occlusion tends to be detected at apparently free space, increase `occlusion.free_space_max` to ignore them.

#### in simple_planning_simulator

intersection_occlusion feature is **not recommended** for use in planning_simulator because the laserscan_based_occupancy_grid_map generates unnatural UNKNOWN cells in 2D manner:

- all the cells behind pedestrians are UNKNOWN
- no ground point clouds are generated

Also many users do not set traffic light information frequently although it is very critical for intersection_occlusion (and in real traffic environment too).

For these reasons, `occlusion.enable` is false by default.

#### on real vehicle / in end-to-end simulator

On real vehicle or in end-to-end simulator like [AWSIM](https://tier4.github.io/AWSIM/) the following pointcloud_based_occupancy_grid_map configuration is highly recommended:

```yaml
scan_origin_frame: "velodyne_top"

grid_map_type: "OccupancyGridMapProjectiveBlindSpot"
OccupancyGridMapProjectiveBlindSpot:
  projection_dz_threshold: 0.01 # [m] for avoiding null division
  obstacle_separation_threshold: 1.0 # [m] fill the interval between obstacles with unknown for this length
```

You should set the top lidar link as the `scan_origin_frame`. In the example it is `velodyne_top`. The method `OccupancyGridMapProjectiveBlindSpot` estimates the FOV by running projective ray-tracing from `scan_origin` to obstacle or up to the ground and filling the cells on the "shadow" of the object as UNKNOWN.

## Flowchart

WIP

```plantuml
@startuml
title modifyPathVelocity
start

:get object polygon;

partition get_objective_polygon {

:get "right-of-way" lanelets from ego lanelet;

:get previous and following ego lanelet from routing graph;

:get all conflicting lanelets with ego lane;

:exclude yield and ego lanelets from conflicting lanelets;

:update conflicting and objective polygon;
}

:get external Input;

:generate stop line;

if (not generate stop line) then (yes)
  stop
else (no)
endif

if (stop line index or pass judge index is same lane as ego?) then (yes)
  stop
else (no)
endif

if (not found closest index?) then (yes)
  stop
else (no)
endif

if (state is GO and over pass judge line and no external stop) then (yes)
  stop
else (no)
endif

if (has collision or is stuck vehicle in intersection?) then (yes)
  :set is_entry_prohibited true;
  :set state from external input;
else (no)
endif

:set state with margin time;

if (current state is same as previous state) then (yes)
  :reset timer;
else if (state is GO->STOP) then (yes)
  :set state as STOP;
  :reset timer;
else if (state is STOP -> GO) then (yes)
  if (start time is not set) then (yes)
    :set start time;
  else(no)
   :calculate duration;
   if(duration is more than margin time)then (yes)
    :set state GO;
    :reset timer;
   endif
  endif
endif

if (state is STOP?) then (yes)
  :set velocity;
  if (stop required?) then (yes)
    :publish stop reason and stop factor;
  endif
else(no)
endif
stop

@enduml
```

## Merge From Private

### Role

When an ego enters a public road from a private road (e.g. a parking lot), it needs to face and stop before entering the public road to make sure it is safe.

This module is activated when there is an intersection at the private area from which the vehicle enters the public road. The stop line is generated both when the goal is in the intersection lane and when the path goes beyond the intersection lane. The basic behavior is the same as the intersection module, but ego must stop once at the stop line.

![merge-from-private](docs/merge_from_private.png)

### Activation Timing

This module is activated when the following conditions are met:

- ego-lane has a `private` tag
- ego-lane has a conflict with other no-private lanelets

### Module Parameters

| Parameter                                   | Type   | Description                     |
| ------------------------------------------- | ------ | ------------------------------- |
| `merge_from_private_road/stop_duration_sec` | double | [m] time margin to change state |

### Known Issue

If ego go over the stop line for a certain distance, then it will not transit from STOP.
