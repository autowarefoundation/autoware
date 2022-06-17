# image_diagnostics

## Purpose

The `image_diagnostics` is a node that check the status of the input raw image.

## Inner-workings / Algorithms

Below figure shows the flowchart of image diagnostics node. Each image is divided into small blocks for block state assessment.

![image diagnostics flowchar ](./image/image_diagnostics_overview.svg)

Each small image block state is assessed as below figure.

![block status decision tree ](./image/block_state_decision.svg)

After all image's blocks state are evaluated, the whole image status is summarized as below.

![whole image state decision tree](./image/image_status_decision.svg)

## Inputs / Outputs

### Input

| Name              | Type                      | Description |
| ----------------- | ------------------------- | ----------- |
| `input/raw_image` | `sensor_msgs::msg::Image` | raw image   |

### Output

| Name                                | Type                                    | Description                           |
| ----------------------------------- | --------------------------------------- | ------------------------------------- |
| `image_diag/debug/gray_image`       | `sensor_msgs::msg::Image`               | gray image                            |
| `image_diag/debug/dft_image`        | `sensor_msgs::msg::Image`               | discrete Fourier transformation image |
| `image_diag/debug/diag_block_image` | `sensor_msgs::msg::Image`               | each block state colorization         |
| `image_diag/image_state_diag`       | `tier4_debug_msgs::msg::Int32Stamped`   | image diagnostics status value        |
| `/diagnostics`                      | `diagnostic_msgs::msg::DiagnosticArray` | diagnostics                           |

## Parameters

## Assumptions / Known limits

- This is proof of concept for image diagnostics and the algorithms still under further improvement.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

- Consider more specific image distortion/occlusion type, for instance raindrop or dust.

- Consider degraded visibility under fog or rain condition from optical point of view
