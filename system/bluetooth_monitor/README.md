# bluetooth_monitor

## Description

This node monitors a Bluetooth connection to a wireless device by using L2ping.<br>
L2ping generates PING echo command on Bluetooth L2CAP layer, and it is able to receive and check echo response from a wireless device.

## Block diagram

L2ping is only allowed for root by default, so this package provides the following approach to minimize security risks as much as possible:

- Provide a small program named `l2ping_service` which performs L2ping and provides wireless device information to `bluetooth_monitor` by using socket programming.
- `bluetooth_monitor` is able to know wireless device information and L2ping status as an unprivileged user since those information are sent by socket communication.

![block_diagram](docs/block_diagram.drawio.svg)

## Output

### <u>bluetooth_monitor: bluetooth_connection</u>

<b>[summary]</b>

| level | message        |
| ----- | -------------- |
| OK    | OK             |
| WARN  | RTT warning    |
| ERROR | Lost           |
|       | Function error |

<b>[values]</b>

| key                        | value (example)                                                         |
| -------------------------- | ----------------------------------------------------------------------- |
| Device [0-9]: Status       | OK / RTT warning / Verify error / Lost / Ping rejected / Function error |
| Device [0-9]: Name         | Wireless Controller                                                     |
| Device [0-9]: Manufacturer | MediaTek, Inc.                                                          |
| Device [0-9]: Address      | AA:BB:CC:DD:EE:FF                                                       |
| Device [0-9]: RTT          | 0.00ms                                                                  |

- The following key will be added when `bluetooth_monitor` reports `Function error`.<br>
  ex.) The `connect` system call failed.

| key (example)         | value (example)           |
| --------------------- | ------------------------- |
| Device [0-9]: connect | No such file or directory |

## Parameters

| Name        | Type   | Default Value | Explanation                                               |
| ----------- | ------ | ------------- | --------------------------------------------------------- |
| `port`      | int    | 7640          | Port number to connect to L2ping service.                 |
| `timeout`   | int    | 5             | Wait timeout seconds for the response.                    |
| `rtt_warn`  | float  | 0.00          | RTT(Round-Trip Time) to generate warn.                    |
| `addresses` | string | \*            | List of bluetooth address of wireless devices to monitor. |

- `rtt_warn`

  - **0.00(zero)**: Disable checking RTT
  - **otherwise**: Check RTT with specified seconds

- `addresses`
  - **\***: All connected devices
  - **AA:BB:CC:DD:EE:FF**: You can specify a device to monitor by setting a Bluetooth address

## Instructions before starting

- You can skip this instructions if you run `l2ping_service` as root user.

1. Assign capability to `l2ping_service` since L2ping requires `cap_net_raw+eip` capability.

   ```sh
   sudo setcap 'cap_net_raw+eip' ./build/bluetooth_monitor/l2ping_service
   ```

2. Run `l2ping_service` and `bluetooth_monitor`.

   ```sh
   ./build/bluetooth_monitor/l2ping_service
   ros2 launch bluetooth_monitor bluetooth_monitor.launch.xml
   ```

## Known limitations and issues

None.
