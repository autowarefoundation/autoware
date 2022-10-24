# System Monitor for Autoware

**Further improvement of system monitor functionality for Autoware.**

## Description

This package provides the following nodes for monitoring system:

- CPU Monitor
- HDD Monitor
- Memory Monitor
- Network Monitor
- NTP Monitor
- Process Monitor
- GPU Monitor
- Voltage Monitor

### Supported architecture

- x86_64
- arm64v8/aarch64

### Operation confirmed platform

- PC system intel core i7
- NVIDIA Jetson AGX Xavier
- Raspberry Pi4 Model B

## How to use

Use colcon build and launch in the same way as other packages.

```sh
colcon build
source install/setup.bash
ros2 launch system_monitor system_monitor.launch.xml
```

CPU and GPU monitoring method differs depending on platform.<br>
CMake automatically chooses source to be built according to build environment.<br>
If you build this package on intel platform, CPU monitor and GPU monitor which run on intel platform are built.

## ROS topics published by system monitor

Every topic is published in 1 minute interval.

- [CPU Monitor](docs/topics_cpu_monitor.md)
- [HDD Monitor](docs/topics_hdd_monitor.md)
- [Mem Monitor](docs/topics_mem_monitor.md)
- [Net Monitor](docs/topics_net_monitor.md)
- [NTP Monitor](docs/topics_ntp_monitor.md)
- [Process Monitor](docs/topics_process_monitor.md)
- [GPU Monitor](docs/topics_gpu_monitor.md)
- [Voltage Monitor](docs/topics_voltage_monitor.md)

[Usage] ✓：Supported, -：Not supported

| Node            | Message                      | Intel | arm64(tegra) | arm64(raspi) | Notes                                                                                                                                                                                           |
| --------------- | ---------------------------- | :---: | :----------: | :----------: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| CPU Monitor     | CPU Temperature              |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | CPU Usage                    |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | CPU Load Average             |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | CPU Thermal Throttling       |   ✓   |      -       |      ✓       |                                                                                                                                                                                                 |
|                 | CPU Frequency                |   ✓   |      ✓       |      ✓       | Notification of frequency only, normally error not generated.                                                                                                                                   |
| HDD Monitor     | HDD Temperature              |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD PowerOnHours             |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD TotalDataWritten         |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD RecoveredError           |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD Usage                    |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD ReadDataRate             |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD WriteDataRate            |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD ReadIOPS                 |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD WriteIOPS                |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | HDD Connection               |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
| Memory Monitor  | Memory Usage                 |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
| Net Monitor     | Network Usage                |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | Network CRC Error            |   ✓   |      ✓       |      ✓       | Warning occurs when the number of CRC errors in the period reaches the threshold value. The number of CRC errors that occur is the same as the value that can be confirmed with the ip command. |
|                 | IP Packet Reassembles Failed |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
| NTP Monitor     | NTP Offset                   |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
| Process Monitor | Tasks Summary                |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | High-load Proc[0-9]          |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
|                 | High-mem Proc[0-9]           |   ✓   |      ✓       |      ✓       |                                                                                                                                                                                                 |
| GPU Monitor     | GPU Temperature              |   ✓   |      ✓       |      -       |                                                                                                                                                                                                 |
|                 | GPU Usage                    |   ✓   |      ✓       |      -       |                                                                                                                                                                                                 |
|                 | GPU Memory Usage             |   ✓   |      -       |      -       |                                                                                                                                                                                                 |
|                 | GPU Thermal Throttling       |   ✓   |      -       |      -       |                                                                                                                                                                                                 |
|                 | GPU Frequency                |   ✓   |      ✓       |      -       | For Intel platform, monitor whether current GPU clock is supported by the GPU.                                                                                                                  |
| Voltage Monitor | CMOS Battery Status          |   ✓   |      -       |      -       | Battery Health for RTC and BIOS -                                                                                                                                                               |

## ROS parameters

See [ROS parameters](docs/ros_parameters.md).

## Notes

### <u>CPU monitor for intel platform</u>

Thermal throttling event can be monitored by reading contents of MSR(Model Specific Register), and accessing MSR is only allowed for root by default, so this package provides the following approach to minimize security risks as much as possible:<br>

- Provide a small program named 'msr_reader' which accesses MSR and sends thermal throttling status to CPU monitor by using socket programming.
- Run 'msr_reader' as a specific user instead of root.
- CPU monitor is able to know the status as an unprivileged user since thermal throttling status is sent by socket communication.

### Instructions before starting

1. Create a user to run 'msr_reader'.

   ```sh
   sudo adduser <username>
   ```

2. Load kernel module 'msr' into your target system.<br>
   The path '/dev/cpu/CPUNUM/msr' appears.

   ```sh
   sudo modprobe msr
   ```

3. Allow user to access MSR with read-only privilege using the Access Control List (ACL).

   ```sh
   sudo setfacl -m u:<username>:r /dev/cpu/*/msr
   ```

4. Assign capability to 'msr_reader' since msr kernel module requires rawio capability.

   ```sh
   sudo setcap cap_sys_rawio=ep install/system_monitor/lib/system_monitor/msr_reader
   ```

5. Run 'msr_reader' as the user you created, and run system_monitor as a generic user.

   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/msr_reader
   ```

### See also

[msr_reader](docs/msr_reader.md)

## <u>HDD Monitor</u>

Generally, S.M.A.R.T. information is used to monitor HDD temperature and life of HDD, and normally accessing disk device node is allowed for root user or disk group.<br>
As with the CPU monitor, this package provides an approach to minimize security risks as much as possible:<br>

- Provide a small program named 'hdd_reader' which accesses S.M.A.R.T. information and sends some items of it to HDD monitor by using socket programming.
- Run 'hdd_reader' as a specific user.
- HDD monitor is able to know some items of S.M.A.R.T. information as an unprivileged user since those are sent by socket communication.

### Instructions before starting

1. Create a user to run 'hdd_reader'.

   ```sh
   sudo adduser <username>
   ```

2. Add user to the disk group.

   ```sh
   sudo usermod -a -G disk <username>
   ```

3. Assign capabilities to 'hdd_reader' since SCSI kernel module requires rawio capability to send ATA PASS-THROUGH (12) command and NVMe kernel module requires admin capability to send Admin Command.

   ```sh
   sudo setcap 'cap_sys_rawio=ep cap_sys_admin=ep' install/system_monitor/lib/system_monitor/hdd_reader
   ```

4. Run 'hdd_reader' as the user you created, and run system_monitor as a generic user.

   ```sh
   su <username>
   install/system_monitor/lib/system_monitor/hdd_reader
   ```

### See also

[hdd_reader](docs/hdd_reader.md)

## <u>GPU Monitor for intel platform</u>

Currently GPU monitor for intel platform only supports NVIDIA GPU whose information can be accessed by NVML API.

Also you need to install CUDA libraries.
For installation instructions for CUDA 10.0, see [NVIDIA CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/archive/10.0/cuda-installation-guide-linux/index.html).

## <u>Voltage monitor for CMOS Battery</u>

Some platforms have built-in batteries for the RTC and CMOS. This node determines the battery status from the result of executing cat /proc/driver/rtc.
Also, if lm-sensors is installed, it is possible to use the results.
However, the return value of sensors varies depending on the chipset, so it is necessary to set a string to extract the corresponding voltage.
It is also necessary to set the voltage for warning and error.
For example, if you want a warning when the voltage is less than 2.9V and an error when it is less than 2.7V.
The execution result of sensors on the chipset nct6106 is as follows, and "in7:" is the voltage of the CMOS battery.

```txt
$ sensors
pch_cannonlake-virtual-0
Adapter: Virtual device
temp1:        +42.0°C

nct6106-isa-0a10
Adapter: ISA adapter
in0:           728.00 mV (min =  +0.00 V, max =  +1.74 V)
in1:             1.01 V  (min =  +0.00 V, max =  +2.04 V)
in2:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in3:             3.34 V  (min =  +0.00 V, max =  +4.08 V)
in4:             1.07 V  (min =  +0.00 V, max =  +2.04 V)
in5:             1.05 V  (min =  +0.00 V, max =  +2.04 V)
in6:             1.67 V  (min =  +0.00 V, max =  +2.04 V)
in7:             3.06 V  (min =  +0.00 V, max =  +4.08 V)
in8:             2.10 V  (min =  +0.00 V, max =  +4.08 V)
fan1:          2789 RPM  (min =    0 RPM)
fan2:             0 RPM  (min =    0 RPM)
```

The setting value of voltage_monitor.param.yaml is as follows.

```yaml
/**:
  ros__parameters:
    cmos_battery_warn: 2.90
    cmos_battery_error: 2.70
    cmos_battery_label: "in7:"
```

The above values of 2.7V and 2.90V are hypothetical. Depending on the motherboard and chipset, the value may vary. However, if the voltage of the lithium battery drops below 2.7V, it is recommended to replace it.
In the above example, the message output to the topic /diagnostics is as follows.
If the voltage < 2.9V then:

```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Low Battery
```

If the voltage < 2.7V then:

```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: Warning
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: Battery Died
```

If neither, then:

```txt
  name: /autoware/system/resource_monitoring/voltage/cmos_battery
  message: OK
  hardware_id: ''
  values:
  - key: 'voltage_monitor: CMOS Battery Status'
    value: OK
```

If the CMOS battery voltage drops less than voltage_error or voltage_warn,It will be a warning.
If the battery runs out, the RTC will stop working when the power is turned off. However, since the vehicle can run, it is not an error. The vehicle will stop when an error occurs, but there is no need to stop immediately.
It can be determined by the value of "Low Battery" or "Battery Died".

## UML diagrams

See [Class diagrams](docs/class_diagrams.md).
See [Sequence diagrams](docs/seq_diagrams.md).
