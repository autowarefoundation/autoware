# Shift Decider

## Purpose

`autoware_shift_decider` is a module to decide shift from ackermann control command.

## Inner-workings / Algorithms

### Flow chart

```plantuml
@startuml
skinparam monochrome true

title update current shift
start
if (absolute target velocity is less than threshold) then (yes)
    :set previous shift;
else(no)
if (target velocity is positive) then (yes)
    :set shift DRIVE;
else
    :set shift REVERSE;
endif
endif
    :publish current shift;
note right
    publish shift for constant interval
end note
stop
@enduml
```

### Algorithms

## Inputs / Outputs

### Input

| Name                  | Type                                  | Description                  |
| --------------------- | ------------------------------------- | ---------------------------- |
| `~/input/control_cmd` | `autoware_control_msgs::msg::Control` | Control command for vehicle. |

### Output

| Name               | Type                                      | Description                        |
| ------------------ | ----------------------------------------- | ---------------------------------- |
| `~output/gear_cmd` | `autoware_vehicle_msgs::msg::GearCommand` | Gear for drive forward / backward. |

## Parameters

none.

## Assumptions / Known limits

TBD.
