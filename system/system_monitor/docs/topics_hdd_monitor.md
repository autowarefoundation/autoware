# ROS topics: CPU Monitor

## <u>HDD Temperature</u>

/diagnostics/hdd_monitor: HDD Temperature

<b>[summary]</b>

| level | message      |
| ----- | ------------ |
| OK    | OK           |
| WARN  | hot          |
| ERROR | critical hot |

<b>[values]</b>

| key                    | value (example)            |
| ---------------------- | -------------------------- |
| HDD [0-9]: status      | OK / hot / critical hot    |
| HDD [0-9]: name        | /dev/nvme0                 |
| HDD [0-9]: model       | SAMSUNG MZVLB1T0HBLR-000L7 |
| HDD [0-9]: serial      | S4EMNF0M820682             |
| HDD [0-9]: temperature | 37.0 DegC                  |

## <u>HDD Usage</u>

/diagnostics/hdd_monitor: HDD Usage

<b>[summary]</b>

| level | message             |
| ----- | ------------------- |
| OK    | OK                  |
| WARN  | low disk space      |
| ERROR | very low disk space |

<b>[values]</b>

| key                   | value (example)                           |
| --------------------- | ----------------------------------------- |
| HDD [0-9]: status     | OK / low disk space / very low disk space |
| HDD [0-9]: filesystem | /dev/nvme0n1p4                            |
| HDD [0-9]: size       | 264G                                      |
| HDD [0-9]: used       | 172G                                      |
| HDD [0-9]: avail      | 749G                                      |
| HDD [0-9]: use        | 69%                                       |
| HDD [0-9]: mounted on | /                                         |
