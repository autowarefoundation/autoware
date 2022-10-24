# ROS topics: Voltage Monitor

"CMOS Battery Status" and "CMOS battery voltage" are exclusive.
Only one or the other is generated.
Which one is generated depends on the value of cmos_battery_label.

## <u>CMOS Battery Status</u>

/diagnostics/voltage_monitor: CMOS Battery Status

<b>[summary]</b>

| level | message      |
| ----- | ------------ |
| OK    | OK           |
| WARN  | Battery Dead |

<b>[values]</b>

| key (example)       | value (example)   |
| ------------------- | ----------------- |
| CMOS battery status | OK / Battery Dead |

\*key: thermal_zone[0-9] for ARM architecture.

## <u>CMOS Battery Voltage</u>

/diagnostics/voltage_monitor: CMOS battery voltage

<b>[summary]</b>

| level | message      |
| ----- | ------------ |
| OK    | OK           |
| WARN  | Low Battery  |
| WARN  | Battery Died |

<b>[values]</b>

| key                  | value (example) |
| -------------------- | --------------- |
| CMOS battery voltage | 3.06            |
