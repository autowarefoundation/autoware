# Side Shift design

(For remote control) Shift the path to left or right according to an external instruction.

## Flowchart

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title callback function of lateral offset input
start

partition onLateralOffset {
:**INPUT** double new_lateral_offset;

if (abs(inserted_lateral_offset_ - new_lateral_offset) < 1e-4) then ( true)
  stop
else ( false)
  if (interval from last request is too short) then ( no)
  else ( yes)
  :requested_lateral_offset_ = new_lateral_offset \n lateral_offset_change_request_ = true;
  endif
stop
@enduml
```

```plantuml -->
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title path generation

start
partition plan {
if (lateral_offset_change_request_ == true \n && \n (shifting_status_ == BEFORE_SHIFT \n || \n shifting_status_ == AFTER_SHIFT)) then ( true)
  partition replace-shift-line {
    if ( shift line is inserted in the path ) then ( yes)
      :erase left shift line;
    else ( no)
    endif
    :calcShiftLines;
    :add new shift lines;
    :inserted_lateral_offset_ = requested_lateral_offset_ \n inserted_shift_lines_ = new_shift_line;
  }
else( false)
endif
stop
@enduml
```

```plantuml
@startuml
skinparam monochrome true
skinparam defaultTextAlignment center
skinparam noteTextAlignment left

title update state

start
partition updateState {
  :last_sp = path_shifter_.getLastShiftLine();
  note left
  get furthest shift lines
  end note
  :calculate max_planned_shift_length;
  note left
  calculate furthest shift length of previous shifted path
  end note
  if (abs(inserted_lateral_offset_ - inserted_shift_line_.end_shift_length) < 1e-4 \n && \n abs(max_planned_shift_length) < 1e-4 \n && \n abs(requested_lateral_offset_) < 1e-4) then ( true)
    :current_state_ = BT::NodeStatus::SUCCESS;
  else (false)
    if (ego's position is behind of shift line's start point) then( yes)
      :shifting_status_ = BEFORE_SHIFT;
    else ( no)
      if ( ego's position is between shift line's start point and end point) then (yes)
        :shifting_status_ = SHIFTING;
      else( no)
        :shifting_status_ = AFTER_SHIFT;
      endif
    endif
    :current_state_ = BT::NodeStatus::RUNNING;
  endif
  stop
}

@enduml
```
