# lidar_apollo_segmentation_tvm {#lidar-apollo-segmentation-tvm-design}

## Design

### Usage {#lidar-apollo-segmentation-tvm-design-usage}

#### Neural network

This package will not build without a neural network for its inference.
The network is provided by the cmake function exported by the tvm_utility package.
See its design page for more information on how to enable downloading pre-compiled networks (by setting the `DOWNLOAD_ARTIFACTS` cmake variable), or how to handle user-compiled networks.

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

Lidar segmentation is based off a core algorithm by [Apollo](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception.md), with modifications from [TierIV] (<https://github.com/tier4/lidar_instance_segmentation_tvm>) for the TVM backend.
