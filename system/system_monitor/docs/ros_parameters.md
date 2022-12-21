# ROS parameters

## <u>CPU Monitor</u>

cpu_monitor:

| Name              | Type  |  Unit   | Default | Notes                                                                                                      |
| :---------------- | :---: | :-----: | :-----: | :--------------------------------------------------------------------------------------------------------- |
| temp_warn         | float |  DegC   |  90.0   | Generates warning when CPU temperature reaches a specified value or higher.                                |
| temp_error        | float |  DegC   |  95.0   | Generates error when CPU temperature reaches a specified value or higher.                                  |
| usage_warn        | float | %(1e-2) |  0.90   | Generates warning when CPU usage reaches a specified value or higher and last for usage_warn_count counts. |
| usage_error       | float | %(1e-2) |  1.00   | Generates error when CPU usage reaches a specified value or higher and last for usage_error_count counts.  |
| usage_warn_count  |  int  |   n/a   |    2    | Generates warning when CPU usage reaches usage_warn value or higher and last for a specified counts.       |
| usage_error_count |  int  |   n/a   |    2    | Generates error when CPU usage reaches usage_error value or higher and last for a specified counts.        |
| load1_warn        | float | %(1e-2) |  0.90   | Generates warning when load average 1min reaches a specified value or higher.                              |
| load5_warn        | float | %(1e-2) |  0.80   | Generates warning when load average 5min reaches a specified value or higher.                              |
| msr_reader_port   |  int  |   n/a   |  7634   | Port number to connect to msr_reader.                                                                      |

## <u>HDD Monitor</u>

hdd_monitor:

&nbsp;&nbsp;disks:

| Name                             |  Type  |       Unit        | Default | Notes                                                                              |
| :------------------------------- | :----: | :---------------: | :-----: | :--------------------------------------------------------------------------------- |
| name                             | string |        n/a        |  none   | The disk name to monitor temperature. (e.g. /dev/sda)                              |
| temp_attribute_id                |  int   |        n/a        |  0xC2   | S.M.A.R.T attribute ID of temperature.                                             |
| temp_warn                        | float  |       DegC        |  55.0   | Generates warning when HDD temperature reaches a specified value or higher.        |
| temp_error                       | float  |       DegC        |  70.0   | Generates error when HDD temperature reaches a specified value or higher.          |
| power_on_hours_attribute_id      |  int   |        n/a        |  0x09   | S.M.A.R.T attribute ID of power-on hours.                                          |
| power_on_hours_warn              |  int   |       Hour        | 3000000 | Generates warning when HDD power-on hours reaches a specified value or higher.     |
| total_data_written_attribute_id  |  int   |        n/a        |  0xF1   | S.M.A.R.T attribute ID of total data written.                                      |
| total_data_written_warn          |  int   | depends on device | 4915200 | Generates warning when HDD total data written reaches a specified value or higher. |
| total_data_written_safety_factor |  int   |      %(1e-2)      |  0.05   | Safety factor of HDD total data written.                                           |
| recovered_error_attribute_id     |  int   |        n/a        |  0xC3   | S.M.A.R.T attribute ID of recovered error.                                         |
| recovered_error_warn             |  int   |        n/a        |    1    | Generates warning when HDD recovered error reaches a specified value or higher.    |
| read_data_rate_warn              | float  |       MB/s        |  360.0  | Generates warning when HDD read data rate reaches a specified value or higher.     |
| write_data_rate_warn             | float  |       MB/s        |  103.5  | Generates warning when HDD write data rate reaches a specified value or higher.    |
| read_iops_warn                   | float  |       IOPS        | 63360.0 | Generates warning when HDD read IOPS reaches a specified value or higher.          |
| write_iops_warn                  | float  |       IOPS        | 24120.0 | Generates warning when HDD write IOPS reaches a specified value or higher.         |

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

| Name                              |     Type     | Unit |  Default   | Notes                                                                                                                                                |
| :-------------------------------- | :----------: | :--: | :--------: | :--------------------------------------------------------------------------------------------------------------------------------------------------- |
| devices                           | list[string] | n/a  |    none    | The name of network interface to monitor. (e.g. eth0, \* for all network interfaces)                                                                 |
| monitor_program                   |    string    | n/a  | greengrass | program name to be monitored by nethogs name.                                                                                                        |
| crc_error_check_duration          |     int      | sec  |     1      | CRC error check duration.                                                                                                                            |
| crc_error_count_threshold         |     int      | n/a  |     1      | Generates warning when count of CRC errors during CRC error check duration reaches a specified value or higher.                                      |
| reassembles_failed_check_duration |     int      | sec  |     1      | IP packet reassembles failed check duration.                                                                                                         |
| reassembles_failed_check_count    |     int      | n/a  |     1      | Generates warning when count of IP packet reassembles failed during IP packet reassembles failed check duration reaches a specified value or higher. |

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

## <u>Voltage Monitor</u>

voltage_monitor:

| Name               |  Type  | Unit | Default | Notes                                                                           |
| :----------------- | :----: | :--: | :-----: | :------------------------------------------------------------------------------ |
| cmos_battery_warn  | float  | volt |   2.9   | Generates warning when voltage of CMOS Battery is lower.                        |
| cmos_battery_error | float  | volt |   2.7   | Generates error when voltage of CMOS Battery is lower.                          |
| cmos_battery_label | string | n/a  |   ""    | voltage string in sensors command outputs. if empty no voltage will be checked. |
