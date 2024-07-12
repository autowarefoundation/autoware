# autoware_component_monitor

The `autoware_component_monitor` package allows monitoring system usage of component containers.
The composable node inside the package is attached to a component container, and it publishes CPU and memory usage of
the container.

## Inputs / Outputs

### Input

None.

### Output

| Name                       | Type                                               | Description            |
| -------------------------- | -------------------------------------------------- | ---------------------- |
| `~/component_system_usage` | `autoware_internal_msgs::msg::ResourceUsageReport` | CPU, Memory usage etc. |

## Parameters

### Core Parameters

{{ json_to_markdown("system/autoware_component_monitor/schema/component_monitor.schema.json") }}

## How to use

Add it as a composable node in your launch file:

```xml

<launch>
  <group>
    <push-ros-namespace namespace="your_namespace"/>
    ...

    <load_composable_node target="$(var container_name)">
      <composable_node pkg="autoware_component_monitor"
                       plugin="autoware::component_monitor::ComponentMonitor"
                       name="component_monitor">
        <param from="$(find-pkg-share autoware_component_monitor)/config/component_monitor.param.yaml"/>
      </composable_node>
    </load_composable_node>

    ...
  </group>
</launch>
```

### Quick testing

You can test the package by running the following command:

```bash
ros2 component load <container_name> autoware_component_monitor autoware::component_monitor::ComponentMonitor -p publish_rate:=10.0 --node-namespace <namespace>

# Example usage
ros2 component load /pointcloud_container autoware_component_monitor autoware::component_monitor::ComponentMonitor -p publish_rate:=10.0 --node-namespace /pointcloud_container
```

## How it works

The package uses the `top` command under the hood.
`top -b -n 1 -E k -p PID` command is run at 10 Hz to get the system usage of the process.

- `-b` activates the batch mode. By default, `top` doesn't exit and prints to stdout periodically. Batch mode allows
  exiting the program.
- `-n` number of times should `top` prints the system usage in batch mode.
- `-p` specifies the PID of the process to monitor.
- `-E k` changes the memory unit in the summary section to KiB.

Here is a sample output:

```text
top - 13:57:26 up  3:14,  1 user,  load average: 1,09, 1,10, 1,04
Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
%Cpu(s):  0,0 us,  0,8 sy,  0,0 ni, 99,2 id,  0,0 wa,  0,0 hi,  0,0 si,  0,0 st
KiB Mem : 65532208 total, 35117428 free, 17669824 used, 12744956 buff/cache
KiB Swap: 39062524 total, 39062524 free,        0 used. 45520816 avail Mem

    PID USER      PR  NI    VIRT    RES    SHR S  %CPU  %MEM     TIME+ COMMAND
   3352 meb       20   0 2905940   1,2g  39292 S   0,0   2,0  23:24.01 awesome
```

We get 5th, 8th fields from the last line, which are RES, %CPU respectively.
