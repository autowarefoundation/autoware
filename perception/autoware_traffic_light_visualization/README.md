# autoware_traffic_light_visualization

## Purpose

The `autoware_traffic_light_visualization` is a package that includes two visualizing nodes:

- **traffic_light_map_visualizer** is a node that shows traffic lights color status and position on rviz as markers.
- **traffic_light_roi_visualizer** is a node that draws the result of traffic light recognition nodes (traffic light status, position and classification probability) on the input image as shown in the following figure and publishes it.

![traffic light roi visualization](./images/roi-visualization.png)

## Inner-workings / Algorithms

## Inputs / Outputs

### traffic_light_map_visualizer

#### Input

| Name                 | Type                                                 | Description              |
| -------------------- | ---------------------------------------------------- | ------------------------ |
| `~/input/tl_state`   | `tier4_perception_msgs::msg::TrafficLightGroupArray` | status of traffic lights |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`              | vector map               |

#### Output

| Name                     | Type                                   | Description                                          |
| ------------------------ | -------------------------------------- | ---------------------------------------------------- |
| `~/output/traffic_light` | `visualization_msgs::msg::MarkerArray` | marker array that indicates status of traffic lights |

### traffic_light_roi_visualizer

#### Input

| Name                          | Type                                               | Description                                             |
| ----------------------------- | -------------------------------------------------- | ------------------------------------------------------- |
| `~/input/tl_state`            | `tier4_perception_msgs::msg::TrafficLightArray`    | status of traffic lights                                |
| `~/input/image`               | `sensor_msgs::msg::Image`                          | the image captured by perception cameras                |
| `~/input/rois`                | `tier4_perception_msgs::msg::TrafficLightRoiArray` | the ROIs detected by `traffic_light_fine_detector`      |
| `~/input/rough/rois` (option) | `tier4_perception_msgs::msg::TrafficLightRoiArray` | the ROIs detected by `traffic_light_map_based_detector` |

#### Output

| Name             | Type                      | Description            |
| ---------------- | ------------------------- | ---------------------- |
| `~/output/image` | `sensor_msgs::msg::Image` | output image with ROIs |

## Parameters

### traffic_light_map_visualizer

None

### traffic_light_roi_visualizer

#### Node Parameters

{{json_to_markdown("perception/autoware_traffic_light_visualization/schema/traffic_light_visualization.schema.json")}}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
