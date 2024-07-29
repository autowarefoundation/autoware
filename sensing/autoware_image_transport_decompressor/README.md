# image_transport_decompressor

## Purpose

The `image_transport_decompressor` is a node that decompresses images.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                       | Type                                | Description      |
| -------------------------- | ----------------------------------- | ---------------- |
| `~/input/compressed_image` | `sensor_msgs::msg::CompressedImage` | compressed image |

### Output

| Name                 | Type                      | Description        |
| -------------------- | ------------------------- | ------------------ |
| `~/output/raw_image` | `sensor_msgs::msg::Image` | decompressed image |

## Parameters

{{ json_to_markdown("sensing/autoware_image_transport_decompressor/schema/image_transport_decompressor.schema.json") }}

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
