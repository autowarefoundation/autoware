# ROS topics: Net Monitor

## <u>Network Connection</u>

/diagnostics/net_monitor: Network Connection

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | no such device |

<b>[values]</b>

| key                   | value (example)     |
| --------------------- | ------------------- |
| Network [0-9]: status | OK / no such device |
| HDD [0-9]: name       | wlp82s0             |

## <u>Network Usage</u>

/diagnostics/net_monitor: Network Usage

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key                           | value (example) |
| ----------------------------- | --------------- |
| Network [0-9]: status         | OK              |
| Network [0-9]: interface name | wlp82s0         |
| Network [0-9]: rx_usage       | 0.00%           |
| Network [0-9]: tx_usage       | 0.00%           |
| Network [0-9]: rx_traffic     | 0.00 MB/s       |
| Network [0-9]: tx_traffic     | 0.00 MB/s       |
| Network [0-9]: capacity       | 400.0 MB/s      |
| Network [0-9]: mtu            | 1500            |
| Network [0-9]: rx_bytes       | 58455228        |
| Network [0-9]: rx_errors      | 0               |
| Network [0-9]: tx_bytes       | 11069136        |
| Network [0-9]: tx_errors      | 0               |
| Network [0-9]: collisions     | 0               |

## <u>Network Traffic</u>

/diagnostics/net_monitor: Network Traffic

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values when specified program is detected]</b>

| key                              | value (example)                             |
| -------------------------------- | ------------------------------------------- |
| nethogs [0-9]: program           | /lambda/greengrassSystemComponents/1384/999 |
| nethogs [0-9]: sent (KB/Sec)     | 1.13574                                     |
| nethogs [0-9]: received (KB/Sec) | 0.261914                                    |

<b>[values when error is occurring]</b>

| key   | value (example)                          |
| ----- | ---------------------------------------- |
| error | execve failed: No such file or directory |

## <u>Network CRC Error</u>

/diagnostics/net_monitor: Network CRC Error

<b>[summary]</b>

| level | message   |
| ----- | --------- |
| OK    | OK        |
| WARN  | CRC error |

<b>[values]</b>

| key                                        | value (example) |
| ------------------------------------------ | --------------- |
| Network [0-9]: interface name              | wlp82s0         |
| Network [0-9]: total rx_crc_errors         | 0               |
| Network [0-9]: rx_crc_errors per unit time | 0               |

## <u>IP Packet Reassembles Failed</u>

/diagnostics/net_monitor: IP Packet Reassembles Failed

<b>[summary]</b>

| level | message            |
| ----- | ------------------ |
| OK    | OK                 |
| WARN  | reassembles failed |

<b>[values]</b>

| key                                     | value (example) |
| --------------------------------------- | --------------- |
| total packet reassembles failed         | 0               |
| packet reassembles failed per unit time | 0               |
