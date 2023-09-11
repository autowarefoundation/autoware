# crosswalk_traffic_light_estimator

## Purpose

`crosswalk_traffic_light_estimator` is a module that estimates pedestrian traffic signals from HDMap and detected vehicle traffic signals.

## Inputs / Outputs

### Input

| Name                                 | Type                                             | Description        |
| ------------------------------------ | ------------------------------------------------ | ------------------ |
| `~/input/vector_map`                 | `autoware_auto_mapping_msgs::msg::HADMapBin`     | vector map         |
| `~/input/route`                      | `autoware_planning_msgs::msg::LaneletRoute`      | route              |
| `~/input/classified/traffic_signals` | `tier4_perception_msgs::msg::TrafficSignalArray` | classified signals |

### Output

| Name                       | Type                                             | Description                                               |
| -------------------------- | ------------------------------------------------ | --------------------------------------------------------- |
| `~/output/traffic_signals` | `tier4_perception_msgs::msg::TrafficSignalArray` | output that contains estimated pedestrian traffic signals |

## Parameters

| Name                          | Type     | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Default value |
| :---------------------------- | :------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `use_last_detect_color`       | `bool`   | If this parameter is `true`, this module estimates pedestrian's traffic signal as RED not only when vehicle's traffic signal is detected as GREEN/AMBER but also when detection results change GREEN/AMBER to UNKNOWN. (If detection results change RED or AMBER to UNKNOWN, this module estimates pedestrian's traffic signal as UNKNOWN.) If this parameter is `false`, this module use only latest detection results for estimation. (Only when the detection result is GREEN/AMBER, this module estimates pedestrian's traffic signal as RED.) | `true`        |
| `last_detect_color_hold_time` | `double` | The time threshold to hold for last detect color.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | `2.0`         |

## Inner-workings / Algorithms

```plantuml

start
:subscribe detected traffic signals & HDMap;
:extract crosswalk lanelets from HDMap;
:extract road lanelets that conflicts crosswalk;
:initialize non_red_lanelets(lanelet::ConstLanelets);
if (Latest detection result is **GREEN** or **AMBER**?) then (yes)
  :push back non_red_lanelets;
else (no)
  if (use_last_detect_color is **true**?) then (yes)
    if (Latest detection result is **UNKNOWN** and last detection result is **GREEN** or **AMBER**?) then (yes)
     :push back non_red_lanelets;
    endif
  endif
endif
if (Is there **STRAIGHT-NON-RED** road lanelet in non_red_lanelets?) then (yes)
  :estimate related pedestrian's traffic signal as **RED**;
else if (Is there both **LEFT-NON-RED** and **RIGHT-NON-RED** road lanelet in non_red_lanelets?) then (yes)
  :estimate related pedestrian's traffic signal as **RED**;
else (no)
  :estimate related pedestrian's traffic signal as **UNKNOWN**;
endif
end

```

If traffic between pedestrians and vehicles is controlled by traffic signals, the crosswalk traffic signal maybe **RED** in order to prevent pedestrian from crossing when the following conditions are satisfied.

### Situation1

- crosswalk conflicts **STRAIGHT** lanelet
- the lanelet refers **GREEN** or **AMBER** traffic signal (The following pictures show only **GREEN** case)

<div align="center">
  <img src="images/straight.drawio.svg" width=80%>
</div>
<div align="center">
  <img src="images/intersection1.svg" width=80%>
</div>

### Situation2

- crosswalk conflicts different turn direction lanelets (STRAIGHT and LEFT, LEFT and RIGHT, RIGHT and STRAIGHT)
- the lanelets refer **GREEN** or **AMBER** traffic signal (The following pictures show only **GREEN** case)

<div align="center">
  <img src="images/intersection2.svg" width=80%>
</div>

## Assumptions / Known limits

## Future extensions / Unimplemented parts
