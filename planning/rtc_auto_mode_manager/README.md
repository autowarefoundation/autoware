# RTC Auto Mode Manager

## Purpose

RTC Auto Mode Manager is a node to approve request to cooperate from behavior planning modules automatically.

## Inputs / Outputs

### Input

| Name                            | Type                        | Description                                |
| ------------------------------- | --------------------------- | ------------------------------------------ |
| `/planning/enable_auto_mode/**` | tier4_rtc_msgs/srv/AutoMode | Service to enable auto mode for the module |

### Output

| Name                                     | Type                        | Description                                |
| ---------------------------------------- | --------------------------- | ------------------------------------------ |
| `/planning/enable_auto_mode/internal/**` | tier4_rtc_msgs/srv/AutoMode | Service to enable auto mode for the module |

## Parameters

| Name                  | Type             | Description                                      |
| :-------------------- | :--------------- | :----------------------------------------------- |
| `module_list`         | List of `string` | Module names managing in `rtc_auto_mode_manager` |
| `default_enable_list` | List of `string` | Module names enabled auto mode at initialization |

## Inner-workings / Algorithms

```plantuml

start
:Read parameters;
:Send enable auto mode service to the module listed in `default_enable_list`;
repeat
  if (Enable auto mode command received?) then (yes)
    :Send enable auto mode command to rtc_interface;
  else (no)
  endif
repeat while (Is node running?) is (yes) not (no)
end

```

## Assumptions / Known limits

## Future extensions / Unimplemented parts
