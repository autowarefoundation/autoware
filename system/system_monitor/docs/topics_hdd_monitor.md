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

<!-- cspell: ignore MZVLB1T0HBLR, S4EMNF0M820682 -->

| key                    | value (example)              |
| ---------------------- | ---------------------------- |
| HDD [0-9]: status      | OK / hot / critical hot      |
| HDD [0-9]: name        | /dev/nvme0                   |
| HDD [0-9]: model       | SAMSUNG MZVLB1T0HBLR-000L7   |
| HDD [0-9]: serial      | S4EMNF0M820682               |
| HDD [0-9]: temperature | 37.0 DegC <br> not available |

## <u>HDD PowerOnHours</u>

/diagnostics/hdd_monitor: HDD PowerOnHours

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | lifetime limit |

<b>[values]</b>

| key                       | value (example)               |
| ------------------------- | ----------------------------- |
| HDD [0-9]: status         | OK / lifetime limit           |
| HDD [0-9]: name           | /dev/nvme0                    |
| HDD [0-9]: model          | PHISON PS5012-E12S-512G       |
| HDD [0-9]: serial         | FB590709182505050767          |
| HDD [0-9]: power on hours | 4834 Hours <br> not available |

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

## <u>HDD RecoveredError</u>

/diagnostics/hdd_monitor: HDD RecoveredError

<b>[summary]</b>

| level | message              |
| ----- | -------------------- |
| OK    | OK                   |
| WARN  | high soft error rate |

<b>[values]</b>

| key                        | value (example)           |
| -------------------------- | ------------------------- |
| HDD [0-9]: status          | OK / high soft error rate |
| HDD [0-9]: name            | /dev/nvme0                |
| HDD [0-9]: model           | PHISON PS5012-E12S-512G   |
| HDD [0-9]: serial          | FB590709182505050767      |
| HDD [0-9]: recovered error | 0 <br> not available      |

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

## <u>HDD ReadDataRate</u>

/diagnostics/hdd_monitor: HDD ReadDataRate

<b>[summary]</b>

| level | message                |
| ----- | ---------------------- |
| OK    | OK                     |
| WARN  | high data rate of read |

<b>[values]</b>

| key                          | value (example)             |
| ---------------------------- | --------------------------- |
| HDD [0-9]: status            | OK / high data rate of read |
| HDD [0-9]: name              | /dev/nvme0                  |
| HDD [0-9]: data rate of read | 0.00 MB/s                   |

## <u>HDD WriteDataRate</u>

/diagnostics/hdd_monitor: HDD WriteDataRate

<b>[summary]</b>

| level | message                 |
| ----- | ----------------------- |
| OK    | OK                      |
| WARN  | high data rate of write |

<b>[values]</b>

| key                           | value (example)              |
| ----------------------------- | ---------------------------- |
| HDD [0-9]: status             | OK / high data rate of write |
| HDD [0-9]: name               | /dev/nvme0                   |
| HDD [0-9]: data rate of write | 0.00 MB/s                    |

## <u>HDD ReadIOPS</u>

/diagnostics/hdd_monitor: HDD ReadIOPS

<b>[summary]</b>

| level | message           |
| ----- | ----------------- |
| OK    | OK                |
| WARN  | high IOPS of read |

<b>[values]</b>

| key                     | value (example)        |
| ----------------------- | ---------------------- |
| HDD [0-9]: status       | OK / high IOPS of read |
| HDD [0-9]: name         | /dev/nvme0             |
| HDD [0-9]: IOPS of read | 0.00 IOPS              |

## <u>HDD WriteIOPS</u>

/diagnostics/hdd_monitor: HDD WriteIOPS

<b>[summary]</b>

| level | message            |
| ----- | ------------------ |
| OK    | OK                 |
| WARN  | high IOPS of write |

<b>[values]</b>

| key                      | value (example)         |
| ------------------------ | ----------------------- |
| HDD [0-9]: status        | OK / high IOPS of write |
| HDD [0-9]: name          | /dev/nvme0              |
| HDD [0-9]: IOPS of write | 0.00 IOPS               |

## <u>HDD Connection</u>

/diagnostics/hdd_monitor: HDD Connection

<b>[summary]</b>

| level | message       |
| ----- | ------------- |
| OK    | OK            |
| WARN  | not connected |

<b>[values]</b>

| key                    | value (example)    |
| ---------------------- | ------------------ |
| HDD [0-9]: status      | OK / not connected |
| HDD [0-9]: name        | /dev/nvme0         |
| HDD [0-9]: mount point | /                  |
