## Blind Spot

### Role

Blind spot check while turning right/left by a dynamic object information and planning a velocity of the start/stop.

![brief](./docs/blind_spot/blind_spot.svg)

### Activation Timing

This function is activated when the lane id of the target path has an intersection label (i.e. the `turn_direction` attribute is `left` or `right`).

### Inner-workings / Algorithms

Sets a stop line, a pass judge line, a detection area and conflict area based on a map information and a self position.

- Stop line : Automatically created based on crossing lane information.

- Pass judge line : A position to judge if stop or not to avoid a rapid brake.

- Detection area : Right/left side area of the self position.

- Conflict area : Right/left side area from the self position to the stop line.

Stop/Go state: When both conditions are met for any of each object, this module state is transited to the "stop" state and insert zero velocity to stop the vehicle.

- Object is on the detection area
- Object’s predicted path is on the conflict area

In order to avoid a rapid stop, the “stop” judgement is not executed after the judgment line is passed.

Once a "stop" is judged, it will not transit to the "go" state until the "go" judgment continues for a certain period in order to prevent chattering of the state (e.g. 2 seconds).

### Module Parameters

| Parameter                       | Type   | Description                                                                 |
| ------------------------------- | ------ | --------------------------------------------------------------------------- |
| `stop_line_margin`              | double | [m] a margin that the vehicle tries to stop before stop_line                |
| `backward_length`               | double | [m] distance from closest path point to the edge of beginning point.        |
| `ignore_width_from_center_line` | double | [m] ignore threshold that vehicle behind is collide with ego vehicle or not |
| `max_future_movement_time`      | double | [s] maximum time for considering future movement of object                  |

### Flowchart

```plantuml
@startuml
title modifyPathVelocity
start

if (ego is turning right or left ?) then (yes)
else (no)
  stop
endif

:calculate pass judge Line;

if (ego vehicle is not after pass judge line?) then (yes)
else (no)
  stop
endif

:check obstacle in blind spot;

if (obstacle found in blind spot?) then (yes)
  :set current state as STOP;
else (no)
  :set current state as GO;
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


if (state is STOP) then (yes)
  :set stop velocity;

  :set stop reason and factor;
endif

stop
@enduml
```
