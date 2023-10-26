# TVM Utility

This is the design document for the `tvm_utility` package. For instructions on how to build the tests for YOLOv2 Tiny,
see the [YOLOv2 Tiny Example Pipeline](tvm-utility-yolo-v2-tiny-tests.md).
For information about where to store test artifacts see the [TVM Utility Artifacts](artifacts/README.md).

## Purpose / Use cases

A set of c++ utilities to help build a TVM based machine learning inference pipeline. The library contains a pipeline
class which helps building the pipeline and a number of utility functions that are common in machine learning.

## Design

The Pipeline Class is a standardized way to write an inference pipeline. The pipeline class contains 3 different stages:
the pre-processor, the inference engine and the post-processor. The TVM implementation of an inference engine stage is
provided.

### API

The pre-processor and post-processor need to be implemented by the user before instantiating the pipeline. You can see example
usage in the example pipeline at `test/yolo_v2_tiny`.

Each stage in the pipeline has a `schedule` function which takes input data as a parameter and return the output data.
Once the pipeline object is created, `pipeline.schedule` is called to run the pipeline.

```{cpp}
int main() {
   create_subscription<sensor_msgs::msg::PointCloud2>("points_raw",
   rclcpp::QoS{1}, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
   {pipeline.schedule(msg);});
}
```

#### Version checking

The `InferenceEngineTVM::version_check` function can be used to check the version of the neural network in use against the range of earliest to latest supported versions.

The `InferenceEngineTVM` class holds the latest supported version, which needs to be updated when the targeted version changes; after having tested the effect of the version change on the packages dependent on this one.

The earliest supported version depends on each package making use of the inference, and so should be defined (and maintained) in those packages.

#### Models

Dependent packages are expected to use the `get_neural_network` cmake function from this package in order to build proper external dependency.

### Error detection and handling

`std::runtime_error` should be thrown whenever an error is encountered. It should be populated with an appropriate text
error description.

### Neural Networks Provider

The neural networks are compiled as part of the
[Model Zoo](https://github.com/autowarefoundation/modelzoo/) CI pipeline and saved to an S3 bucket.

The `get_neural_network` function creates an abstraction for the artifact management.
Users should check if model configuration header file is under "data/user/${MODEL_NAME}/". Otherwise, nothing happens and compilation of the package will be skipped.

The structure inside of the source directory of the package making use of the function is as follow:

```{text}
.
├── data
│   └── models
│       ├── ${MODEL 1}
│       │   └── inference_engine_tvm_config.hpp
│       ├── ...
│       └── ${MODEL ...}
│           └── ...
```

The `inference_engine_tvm_config.hpp` file needed for compilation by dependent packages should be available under "data/models/${MODEL_NAME}/inference_engine_tvm_config.hpp".
Dependent packages can use the cmake `add_dependencies` function with the name provided in the `DEPENDENCY` output parameter of `get_neural_network` to ensure this file is created before it gets used.

The other `deploy_*` files are installed to "models/${MODEL_NAME}/" under the `share` directory of the package.

The other model files should be stored in autoware_data folder under package folder with the structure:

```{text}
$HOME/autoware_data
|     └──${package}
|        └──models
|           ├── ${MODEL 1}
|           |    ├── deploy_graph.json
|           |    ├── deploy_lib.so
|           |    └── deploy_param.params
|           ├── ...
|           └── ${MODEL ...}
|                └── ...
```

#### Inputs / Outputs

Outputs:

- `get_neural_network` cmake function; create proper external dependency for a package with use of the model provided by the user.

In/Out:

- The `DEPENDENCY` argument of `get_neural_network` can be checked for the outcome of the function.
  It is an empty string when the neural network wasn't provided by the user.

## Security considerations

### Pipeline

Both the input and output are controlled by the same actor, so the following security concerns are out-of-scope:

- Spoofing
- Tampering

Leaking data to another actor would require a flaw in TVM or the host operating system that allows arbitrary memory to
be read, a significant security flaw in itself. This is also true for an external actor operating the pipeline early:
only the object that initiated the pipeline can run the methods to receive its output.

A Denial-of-Service attack could make the target hardware unusable for other pipelines but would require being able to
run code on the CPU, which would already allow a more severe Denial-of-Service attack.

No elevation of privilege is required for this package.

### Network provider

The pre-compiled networks are downloaded from an S3 bucket and are under threat of spoofing,
tampering and denial of service.
Spoofing is mitigated by using an https connection.
Mitigations for tampering and denial of service are left to AWS.

The user-provided networks are installed as they are on the host system.
The user is in charge of securing the files they provide with regard to information disclosure.

## Future extensions / Unimplemented parts

Future packages will use tvm_utility as part of the perception stack to run machine learning workloads.

## Related issues

<https://github.com/autowarefoundation/autoware/discussions/2557>
