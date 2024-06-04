# yabloc_common

This package contains some executable nodes related to map. Also, This provides some yabloc common library.

- [ground_server](#ground_server)
- [ll2_decomposer](#ll2_decomposer)

## ground_server

### Purpose

It estimates the height and tilt of the ground from lanelet2.

### Input / Outputs

#### Input

| Name               | Type                                    | Description         |
| ------------------ | --------------------------------------- | ------------------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | vector map          |
| `input/pose`       | `geometry_msgs::msg::PoseStamped`       | estimated self pose |

#### Output

| Name                    | Type                               | Description                                                                     |
| ----------------------- | ---------------------------------- | ------------------------------------------------------------------------------- |
| `output/ground`         | `std_msgs::msg::Float32MultiArray` | estimated ground parameters. it contains x, y, z, normal_x, normal_y, normal_z. |
| `output/ground_markers` | `visualization_msgs::msg::Marker`  | visualization of estimated ground plane                                         |
| `output/ground_status`  | `std_msgs::msg::String`            | status log of ground plane estimation                                           |
| `output/height`         | `std_msgs::msg::Float32`           | altitude                                                                        |
| `output/near_cloud`     | `sensor_msgs::msg::PointCloud2`    | point cloud extracted from lanelet2 and used for ground tilt estimation         |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_common/schema/ground_server.schema.json") }}

## ll2_decomposer

### Purpose

This node extracts the elements related to the road surface markings and yabloc from lanelet2.

### Input / Outputs

#### Input

| Name               | Type                                    | Description |
| ------------------ | --------------------------------------- | ----------- |
| `input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin` | vector map  |

#### Output

| Name                       | Type                                   | Description                                   |
| -------------------------- | -------------------------------------- | --------------------------------------------- |
| `output/ll2_bounding_box`  | `sensor_msgs::msg::PointCloud2`        | bounding boxes extracted from lanelet2        |
| `output/ll2_road_marking`  | `sensor_msgs::msg::PointCloud2`        | road surface markings extracted from lanelet2 |
| `output/ll2_sign_board`    | `sensor_msgs::msg::PointCloud2`        | traffic sign boards extracted from lanelet2   |
| `output/sign_board_marker` | `visualization_msgs::msg::MarkerArray` | visualized traffic sign boards                |

### Parameters

{{ json_to_markdown("localization/yabloc/yabloc_common/schema/ll2_decomposer.schema.json") }}
