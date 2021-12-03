# ROS topics: Process Monitor

## <u>Tasks Summary</u>

/diagnostics/process_monitor: Tasks Summary

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key      | value (example) |
| -------- | --------------- |
| total    | 409             |
| running  | 2               |
| sleeping | 321             |
| stopped  | 0               |
| zombie   | 0               |

## <u>High-load Proc[0-9]</u>

/diagnostics/process_monitor: High-load Proc[0-9]

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key     | value (example)          |
| ------- | ------------------------ |
| COMMAND | /usr/lib/firefox/firefox |
| %CPU    | 37.5                     |
| %MEM    | 2.1                      |
| PID     | 14062                    |
| USER    | autoware                 |
| PR      | 20                       |
| NI      | 0                        |
| VIRT    | 3461152                  |
| RES     | 669052                   |
| SHR     | 481208                   |
| S       | S                        |
| TIME+   | 23:57.49                 |

## <u>High-mem Proc[0-9]</u>

/diagnostics/process_monitor: High-mem Proc[0-9]

<b>[summary]</b>

| level | message |
| ----- | ------- |
| OK    | OK      |

<b>[values]</b>

| key     | value (example)                                 |
| ------- | ----------------------------------------------- |
| COMMAND | /snap/multipass/1784/usr/bin/qemu-system-x86_64 |
| %CPU    | 0                                               |
| %MEM    | 2.5                                             |
| PID     | 1565                                            |
| USER    | root                                            |
| PR      | 20                                              |
| NI      | 0                                               |
| VIRT    | 3722320                                         |
| RES     | 812432                                          |
| SHR     | 20340                                           |
| S       | S                                               |
| TIME+   | 0:22.84                                         |
