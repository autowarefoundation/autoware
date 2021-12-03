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

[Usage] ✓：Supported, -：Not supported

| Node            | Message                | Intel | arm64(tegra) | arm64(raspi) | Notes                                                         |
| --------------- | ---------------------- | :---: | :----------: | :----------: | ------------------------------------------------------------- |
| CPU Monitor     | CPU Temperature        |   ✓   |      ✓       |      ✓       |                                                               |
|                 | CPU Usage              |   ✓   |      ✓       |      ✓       |                                                               |
|                 | CPU Load Average       |   ✓   |      ✓       |      ✓       |                                                               |
|                 | CPU Thermal Throttling |   ✓   |      -       |      ✓       |                                                               |
|                 | CPU Frequency          |   ✓   |      ✓       |      ✓       | Notification of frequency only, normally error not generated. |
| HDD Monitor     | HDD Temperature        |   ✓   |      ✓       |      ✓       |                                                               |
|                 | HDD Usage              |   ✓   |      ✓       |      ✓       |                                                               |
| Memory Monitor  | Memory Usage           |   ✓   |      ✓       |      ✓       |                                                               |
| Net Monitor     | Network Usage          |   ✓   |      ✓       |      ✓       |                                                               |
| NTP Monitor     | NTP Offset             |   ✓   |      ✓       |      ✓       |                                                               |
| Process Monitor | Tasks Summary          |   ✓   |      ✓       |      ✓       |                                                               |
|                 | High-load Proc[0-9]    |   ✓   |      ✓       |      ✓       |                                                               |
|                 | High-mem Proc[0-9]     |   ✓   |      ✓       |      ✓       |                                                               |
| GPU Monitor     | GPU Temperature        |   ✓   |      ✓       |      -       |                                                               |
|                 | GPU Usage              |   ✓   |      ✓       |      -       |                                                               |
|                 | GPU Memory Usage       |   ✓   |      -       |      -       |                                                               |
|                 | GPU Thermal Throttling |   ✓   |      -       |      -       |                                                               |
|                 | GPU Frequency          |   -   |      ✓       |      -       |                                                               |

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

Generally, S.M.A.R.T. information is used to monitor HDD temperature, and normally accessing disk device node is allowed for root user or disk group.<br>
As with the CPU monitor, this package provides an approach to minimize security risks as much as possible:<br>

- Provide a small program named 'hdd_reader' which accesses S.M.A.R.T. information and sends HDD temperature to HDD monitor by using socket programming.
- Run 'hdd_reader' as a specific user.
- HDD monitor is able to know HDD temperature as an unprivileged user since HDD temperature is sent by socket communication.

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

## UML diagrams

See [Class diagrams](docs/class_diagrams.md).
See [Sequence diagrams](docs/seq_diagrams.md).
