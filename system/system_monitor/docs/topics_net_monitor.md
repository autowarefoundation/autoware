# ROS topics: Net Monitor

## <u>Network Usage</u>

/diagnostics/cpu_monitor: Network Usage

<b>[summary]</b>

| level | message   |
| ----- | --------- |
| OK    | OK        |
| WARN  | high load |
| ERROR | down      |

<b>[values]</b>

| key                           | value (example)       |
| ----------------------------- | --------------------- |
| Network [0-9]: status         | OK / high load / down |
| Network [0-9]: interface name | wlp82s0               |
| Network [0-9]: rx_usage       | 0.00%                 |
| Network [0-9]: tx_usage       | 0.00%                 |
| Network [0-9]: rx_traffic     | 0.00 MB/s             |
| Network [0-9]: tx_traffic     | 0.00 MB/s             |
| Network [0-9]: capacity       | 400.0 MB/s            |
| Network [0-9]: mtu            | 1500                  |
| Network [0-9]: rx_bytes       | 58455228              |
| Network [0-9]: rx_errors      | 0                     |
| Network [0-9]: tx_bytes       | 11069136              |
| Network [0-9]: tx_errors      | 0                     |
| Network [0-9]: collisions     | 0                     |
