# system_launch

## Structure

![system_launch](./system_launch.drawio.svg)

## Package Dependencies

Please see `<exec_depend>` in `package.xml`.

## Usage

```xml
  <include file="$(find-pkg-share system_launch)/launch/system.launch.xml">
    <arg name="run_mode" value="online"/>
    <arg name="sensor_model" value="SENSOR_MODEL"/>
  </include>
```

The sensing configuration parameters used in autoware_error_monitor are loaded from "config/diagnostic_aggregator/sensor_kit.param.yaml" in the "`SENSOR_MODEL`\_description" package.
