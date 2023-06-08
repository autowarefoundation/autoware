### No Stopping Area

#### Role

This module plans to avoid stop in 'no stopping area`.

![brief](./docs/no-stopping-area.svg)

- PassThrough case
  - if ego vehicle go through pass judge point, then ego vehicle can't stop with maximum jerk and acceleration, so this module doesn't insert stop velocity. In this case override or external operation is necessary.
- STOP case
  - If there is a stuck vehicle or stop velocity around `no_stopping_area`, then vehicle stops inside `no_stopping_area` so this module makes stop velocity in front of `no_stopping_area`
- GO case
  - else

### Limitation

This module allows developers to design vehicle velocity in `no_stopping_area` module using specific rules. Once ego vehicle go through pass through point, ego vehicle does't insert stop velocity and does't change decision from GO. Also this module only considers dynamic object in order to avoid unnecessarily stop.

#### ModelParameter

| Parameter                    | Type   | Description                                                         |
| ---------------------------- | ------ | ------------------------------------------------------------------- |
| `state_clear_time`           | double | [s] time to clear stop state                                        |
| `stuck_vehicle_vel_thr`      | double | [m/s] vehicles below this velocity are considered as stuck vehicle. |
| `stop_margin`                | double | [m] margin to stop line at no stopping area                         |
| `dead_line_margin`           | double | [m] if ego pass this position GO                                    |
| `stop_line_margin`           | double | [m] margin to auto-gen stop line at no stopping area                |
| `detection_area_length`      | double | [m] length of searching polygon                                     |
| `stuck_vehicle_front_margin` | double | [m] obstacle stop max distance                                      |

#### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

if (ego path has "no stopping area" ?) then (yes)
else (no)
  stop
endif

partition pass_through_condition {
if (ego vehicle is not after dead line?) then (yes)
else (no)
  stop
endif
if (ego vehicle is stoppable before stop line consider jerk limit?) then (yes)
else (no)
  stop
endif
}
note right
  - ego vehicle is already over dead line(1.0[m] forward stop line) Do Not Stop.
  - "pass through or not" considering jerk limit is judged only once to avoid chattering.
end note

:generate ego "stuck_vehicle_detect_area" polygon;
note right
"stuck_vehicle_detect_area" polygon includes space of
 vehicle_length + obstacle_stop_max_distance
 after "no stopping area"
end note

:generate ego "stop_line_detect_area" polygon;
note right
"stop_line_detect_area" polygon includes space of
 vehicle_length + margin
 after "no stopping area"
end note

:set current judgement as GO;
if (Is stuck vehicle inside "stuck_vehicle_detect_area" polygon?) then (yes)
note right
only consider stuck vehicle following condition.
- below velocity 3.0 [m/s]
- semantic type of car bus truck or motorbike
only consider stop line as following condition.
- low velocity that is in path with lane id is considered.
end note
if (Is stop line inside "stop_line_detect_area" polygon?) then (yes)
  :set current judgement as STOP;
endif
endif

partition set_state_with_margin_time {

if (current judgement is same as previous state) then (yes)
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
  else(no)
   endif
  endif
else(no)
endif

}

note right
  - it takes 2 seconds to change state from STOP -> GO
  - it takes 0 seconds to change state from GO -> STOP
  - reset timer if no state change
end note

if (state is STOP) then (yes)
  :set stop velocity;
  :set stop reason and factor;
  else(no)
endif
stop


@enduml
```
