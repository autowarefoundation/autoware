# lidar_apollo_segmentation_tvm

## Design

### Usage

#### Neural network

This package will not run without a neural network for its inference.
The network is provided by ansible script during the installation of Autoware or can be downloaded manually according to [Manual Downloading](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/artifacts).
This package uses 'get_neural_network' function from tvm_utility package to create and provide proper dependency.
See its design page for more information on how to handle user-compiled networks.

#### Backend

The backend used for the inference can be selected by setting the `lidar_apollo_segmentation_tvm_BACKEND` cmake variable.
The current available options are `llvm` for a CPU backend, and `vulkan` for a GPU backend.
It defaults to `llvm`.

### Convolutional Neural Networks (CNN) Segmentation

See the [original design](https://github.com/ApolloAuto/apollo/blob/3422a62ce932cb1c0c269922a0f1aa59a290b733/docs/specs/3d_obstacle_perception.md#convolutional-neural-networks-cnn-segmentation) by Apollo.
The paragraph of interest goes up to, but excluding, the "MinBox Builder" paragraph.
This package instead relies on further processing by a dedicated shape estimator.

Note: the parameters described in the original design have been modified and are out of date.

### Inputs / Outputs / API

The package exports a boolean `lidar_apollo_segmentation_tvm_BUILT` cmake variable.

## Reference

Lidar segmentation is based off a core algorithm by [Apollo](https://github.com/ApolloAuto/apollo/blob/r6.0.0/docs/specs/3d_obstacle_perception.md), with modifications from [TIER IV] (<https://github.com/tier4/lidar_instance_segmentation_tvm>) for the TVM backend.
