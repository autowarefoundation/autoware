# ROS topics: Net Monitor

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

<b>[values] program</b>

| key                              | value (example)                             |
| -------------------------------- | ------------------------------------------- |
| nethogs [0-9]: PROGRAM           | /lambda/greengrassSystemComponents/1384/999 |
| nethogs [0-9]: SENT (KB/Sec)     | 1.13574                                     |
| nethogs [0-9]: RECEIVED (KB/Sec) | 0.261914                                    |

<b>[values] all</b>

| key                   | value (example)                                                |
| --------------------- | -------------------------------------------------------------- |
| nethogs: all (KB/Sec) | python3.7/1520/999 0.274414 0.354883                           |
|                       | /lambda/greengrassSystemComponents/1299/999 0.487305 0.0966797 |
|                       | sshd: muser@pts/5/15917/1002 0.396094 0.0585938                |
|                       | /usr/bin/python3.7/2371/999 0 0                                |
|                       | /greengrass/ggc/packages/1.10.0/bin/daemon/906/0 0 0           |
|                       | python3.7/4362/999 0 0                                         |
|                       | unknown TCP/0/0 0 0                                            |

<b>[values] error</b>

| key   | value (example)                                       |
| ----- | ----------------------------------------------------- |
| error | [nethogs -t] execve failed: No such file or directory |

## <u>Network CRC Error</u>

/diagnostics/net_monitor: Network CRC Error

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

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
