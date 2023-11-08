<!--
  Copyright 2021-2023 Arm Ltd., the Autoware Foundation

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-->

# lidar_apollo_segmentation_tvm_nodes

## Purpose / Use cases

An alternative to Euclidean clustering.
This node detects and labels foreground obstacles (e.g. cars, motorcycles, pedestrians) from a point
cloud, using a neural network.

## Design

See the design of the algorithm in the core (lidar_apollo_segmentation_tvm) package's design documents.

### Usage

`lidar_apollo_segmentation_tvm` and `lidar_apollo_segmentation_tvm_nodes` will not work without a neural network.
See the lidar_apollo_segmentation_tvm usage for more information.

### Assumptions / Known limits

The original node from Apollo has a Region Of Interest (ROI) filter.
This has the benefit of working with a filtered point cloud that includes only the points inside the
ROI (i.e., the drivable road and junction areas) with most of the background obstacles removed (such
as buildings and trees around the road region).
Not having this filter may negatively impact performance.

### Inputs / Outputs / API

#### Inputs

The input are non-ground points as a PointCloud2 message from the sensor_msgs package.

#### Outputs

The output is a [DetectedObjectsWithFeature](https://github.com/tier4/tier4_autoware_msgs/blob/tier4/universe/tier4_perception_msgs/msg/object_recognition/DetectedObjectsWithFeature.msg).

#### Parameters

{{ json_to_markdown("perception/lidar_apollo_segmentation_tvm_nodes/schema/lidar_apollo_segmentation_tvm_nodes.schema.json") }}

### Error detection and handling

Abort and warn when the input frame can't be converted to `base_link`.

### Security considerations

Both the input and output are controlled by the same actor, so the following security concerns are
out-of-scope:

- Spoofing
- Tampering

Leaking data to another actor would require a flaw in TVM or the host operating system that allows
arbitrary memory to be read, a significant security flaw in itself.
This is also true for an external actor operating the pipeline early: only the object that initiated
the pipeline can run the methods to receive its output.

A Denial-of-Service attack could make the target hardware unusable for other pipelines but would
require being able to run code on the CPU, which would already allow a more severe Denial-of-Service
attack.

No elevation of privilege is required for this package.

## Future extensions / Unimplemented parts

## Related issues

- #226: Autoware.Auto Neural Networks Inference Architecture Design
