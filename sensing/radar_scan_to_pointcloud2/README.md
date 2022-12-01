# radar_scan_to_pointcloud2

## radar_scan_to_pointcloud2_node

- Convert from `radar_msgs::msg::RadarScan` to `sensor_msgs::msg::PointCloud2`
- Calculation cost O(n)
  - n: The number of radar return

### Input topics

| Name        | Type                       | Description |
| ----------- | -------------------------- | ----------- |
| input/radar | radar_msgs::msg::RadarScan | RadarScan   |

### Output topics

| Name                        | Type                          | Description                                                       |
| --------------------------- | ----------------------------- | ----------------------------------------------------------------- |
| output/amplitude_pointcloud | sensor_msgs::msg::PointCloud2 | PointCloud2 radar pointcloud whose intensity is amplitude.        |
| output/doppler_pointcloud   | sensor_msgs::msg::PointCloud2 | PointCloud2 radar pointcloud whose intensity is doppler velocity. |

### Parameters

| Name                         | Type | Description                                                                               |
| ---------------------------- | ---- | ----------------------------------------------------------------------------------------- |
| publish_amplitude_pointcloud | bool | Whether publish radar pointcloud whose intensity is amplitude. Default is `true`.         |
| publish_doppler_pointcloud   | bool | Whether publish radar pointcloud whose intensity is doppler velocity. Default is `false`. |

### How to launch

```sh
ros2 launch radar_scan_to_pointcloud2 radar_scan_to_pointcloud2.launch.xml
```
