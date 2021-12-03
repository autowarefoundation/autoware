# livox_tag_filter

## Purpose

The `livox_tag_filter` is a node that removes noise from pointcloud by using the following tags:

- Point property based on spatial position
- Point property based on intensity
- Return number

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name      | Type                            | Description      |
| --------- | ------------------------------- | ---------------- |
| `~/input` | `sensor_msgs::msg::PointCloud2` | reference points |

### Output

| Name       | Type                            | Description     |
| ---------- | ------------------------------- | --------------- |
| `~/output` | `sensor_msgs::msg::PointCloud2` | filtered points |

## Parameters

### Node Parameters

| Name          | Type            | Description                            |
| ------------- | --------------- | -------------------------------------- |
| `ignore_tags` | vector<int64_t> | ignored tags (See the following table) |

### Tag Parameters

| Bit | Description                              | Options                                    |
| --- | ---------------------------------------- | ------------------------------------------ |
| 0~1 | Point property based on spatial position | 00: Normal                                 |
|     |                                          | 01: High confidence level of the noise     |
|     |                                          | 10: Moderate confidence level of the noise |
|     |                                          | 11: Low confidence level of the noise      |
| 2~3 | Point property based on intensity        | 00: Normal                                 |
|     |                                          | 01: High confidence level of the noise     |
|     |                                          | 10: Moderate confidence level of the noise |
|     |                                          | 11: Reserved                               |
| 4~5 | Return number                            | 00: return 0                               |
|     |                                          | 01: return 1                               |
|     |                                          | 10: return 2                               |
|     |                                          | 11: return 3                               |
| 6~7 | Reserved                                 |                                            |

You can download more detail description about the livox from external link [1].

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

[1] <https://www.livoxtech.com/downloads>

## (Optional) Future extensions / Unimplemented parts
