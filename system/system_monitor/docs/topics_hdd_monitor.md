# ROS topics: HDD Monitor

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

## <u>HDD PowerOnHours</u>

/diagnostics/hdd_monitor: HDD PowerOnHours

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | lifetime limit |

<b>[values]</b>

| key                       | value (example)         |
| ------------------------- | ----------------------- |
| HDD [0-9]: status         | OK / lifetime limit     |
| HDD [0-9]: name           | /dev/nvme0              |
| HDD [0-9]: model          | PHISON PS5012-E12S-512G |
| HDD [0-9]: serial         | FB590709182505050767    |
| HDD [0-9]: power on hours | 4834 Hours              |

## <u>HDD TotalDataWritten</u>

/diagnostics/hdd_monitor: HDD TotalDataWritten

<b>[summary]</b>

| level | message         |
| ----- | --------------- |
| OK    | OK              |
| WARN  | warranty period |

<b>[values]</b>

| key                           | value (example)              |
| ----------------------------- | ---------------------------- |
| HDD [0-9]: status             | OK / warranty period         |
| HDD [0-9]: name               | /dev/nvme0                   |
| HDD [0-9]: model              | PHISON PS5012-E12S-512G      |
| HDD [0-9]: serial             | FB590709182505050767         |
| HDD [0-9]: total data written | 146295330 <br> not available |

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
