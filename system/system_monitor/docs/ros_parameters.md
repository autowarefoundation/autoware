# ROS parameters

## <u>CPU Monitor</u>

cpu_monitor:

| Name            | Type  |  Unit   | Default | Notes                                                                         |
| :-------------- | :---: | :-----: | :-----: | :---------------------------------------------------------------------------- |
| temp_warn       | float |  DegC   |  90.0   | Generates warning when CPU temperature reaches a specified value or higher.   |
| temp_error      | float |  DegC   |  95.0   | Generates error when CPU temperature reaches a specified value or higher.     |
| usage_warn      | float | %(1e-2) |  0.90   | Generates warning when CPU usage reaches a specified value or higher.         |
| usage_error     | float | %(1e-2) |  1.00   | Generates error when CPU usage reaches a specified value or higher.           |
| load1_warn      | float | %(1e-2) |  0.90   | Generates warning when load average 1min reaches a specified value or higher. |
| load5_warn      | float | %(1e-2) |  0.80   | Generates warning when load average 5min reaches a specified value or higher. |
| msr_reader_port |  int  |   n/a   |  7634   | Port number to connect to msr_reader.                                         |

## <u>HDD Monitor</u>

hdd_monitor:

&nbsp;&nbsp;disks:

| Name       |  Type  | Unit | Default | Notes                                                                       |
| :--------- | :----: | :--: | :-----: | :-------------------------------------------------------------------------- |
| name       | string | n/a  |  none   | The disk name to monitor temperature. (e.g. /dev/sda)                       |
| temp_error | float  | DegC |  55.0   | Generates warning when HDD temperature reaches a specified value or higher. |
| temp_error | float  | DegC |  70.0   | Generates error when HDD temperature reaches a specified value or higher.   |

hdd_monitor:

| Name            | Type  |  Unit   | Default | Notes                                                                  |
| :-------------- | :---: | :-----: | :-----: | :--------------------------------------------------------------------- |
| hdd_reader_port |  int  |   n/a   |  7635   | Port number to connect to hdd_reader.                                  |
| usage_warn      | float | %(1e-2) |  0.95   | Generates warning when disk usage reaches a specified value or higher. |
| usage_error     | float | %(1e-2) |  0.99   | Generates error when disk usage reaches a specified value or higher.   |

## <u>Memory Monitor</u>

mem_monitor:

| Name        | Type  |  Unit   | Default | Notes                                                                             |
| :---------- | :---: | :-----: | :-----: | :-------------------------------------------------------------------------------- |
| usage_warn  | float | %(1e-2) |  0.95   | Generates warning when physical memory usage reaches a specified value or higher. |
| usage_error | float | %(1e-2) |  0.99   | Generates error when physical memory usage reaches a specified value or higher.   |

## <u>Net Monitor</u>

net_monitor:

| Name       |     Type     |  Unit   | Default | Notes                                                                                |
| :--------- | :----------: | :-----: | :-----: | :----------------------------------------------------------------------------------- |
| devices    | list[string] |   n/a   |  none   | The name of network interface to monitor. (e.g. eth0, \* for all network interfaces) |
| usage_warn |    float     | %(1e-2) |  0.95   | Generates warning when network usage reaches a specified value or higher.            |

## <u>NTP Monitor</u>

ntp_monitor:

| Name         |  Type  | Unit |    Default     | Notes                                                                                     |
| :----------- | :----: | :--: | :------------: | :---------------------------------------------------------------------------------------- |
| server       | string | n/a  | ntp.ubuntu.com | The name of NTP server to synchronize date and time. (e.g. ntp.nict.jp for Japan)         |
| offset_warn  | float  | sec  |      0.1       | Generates warning when NTP offset reaches a specified value or higher. (default is 100ms) |
| offset_error | float  | sec  |      5.0       | Generates warning when NTP offset reaches a specified value or higher. (default is 5sec)  |

## <u>Process Monitor</u>

process_monitor:

| Name         | Type | Unit | Default | Notes                                                                           |
| :----------- | :--: | :--: | :-----: | :------------------------------------------------------------------------------ |
| num_of_procs | int  | n/a  |    5    | The number of processes to generate High-load Proc[0-9] and High-mem Proc[0-9]. |

## <u>GPU Monitor</u>

gpu_monitor:

| Name               | Type  |  Unit   | Default | Notes                                                                        |
| :----------------- | :---: | :-----: | :-----: | :--------------------------------------------------------------------------- |
| temp_warn          | float |  DegC   |  90.0   | Generates warning when GPU temperature reaches a specified value or higher.  |
| temp_error         | float |  DegC   |  95.0   | Generates error when GPU temperature reaches a specified value or higher.    |
| gpu_usage_warn     | float | %(1e-2) |  0.90   | Generates warning when GPU usage reaches a specified value or higher.        |
| gpu_usage_error    | float | %(1e-2) |  1.00   | Generates error when GPU usage reaches a specified value or higher.          |
| memory_usage_warn  | float | %(1e-2) |  0.90   | Generates warning when GPU memory usage reaches a specified value or higher. |
| memory_usage_error | float | %(1e-2) |  1.00   | Generates error when GPU memory usage reaches a specified value or higher.   |
