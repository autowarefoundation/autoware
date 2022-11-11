# TVM Utility {#tvm-utility-design}

This is the design document for the `tvm_utility` package. For instructions on how to build the tests for YOLOv2 Tiny,
see the @subpage tvm-utility-yolo-v2-tiny-tests. For information about where to store test artifacts see the @subpage tvm-utility-artifacts-readme.

## Purpose / Use cases

A set of c++ utilities to help build a TVM based machine learning inference pipeline. The library contains a pipeline
class which helps building the pipeline and a number of utility functions that are common in machine learning.

## Design

The Pipeline Class is a standardized way to write an inference pipeline. The pipeline class contains 3 different stages:
the pre-processor, the inference engine and the post-processor. The TVM implementation of an inference engine stage is
provided.

### API

The pre-processor and post-processor need to be implemented by the user before instantiating the pipeline. You can see example
usage in this [example_pipeline](../test/yolo_v2_tiny).

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

Dependent packages are expected to use the `get_neural_network` cmake function from this package in order to get the compiled TVM models.

### Error detection and handling

`std::runtime_error` should be thrown whenever an error is encountered. It should be populated with an appropriate text
error description.

### Neural Networks Provider

This package also provides a utility to get pre-compiled neural networks to packages using them for their inference.

The neural networks are compiled as part of the
[Model Zoo](https://github.com/autowarefoundation/modelzoo/) CI pipeline and saved to an S3 bucket.
This package exports cmake variables and functions for ease of access to those neural networks.

The `get_neural_network` function creates an abstraction for the artifact management.
The artifacts are saved under the source directory of the package making use of the function; under "data/".
Priority is given to user-provided files, under "data/user/${MODEL_NAME}/".
If there are no user-provided files, the function tries to reuse previously-downloaded artifacts.
If there are no previously-downloaded artifacts, and if the `DOWNLOAD_ARTIFACTS` cmake variable is set, they will be downloaded from the bucket.
Otherwise, nothing happens.

The structure inside of the source directory of the package making use of the function is as follow:

```{text}
.
├── data
│   ├── downloads
│   │   ├── ${MODEL 1}-${ARCH 1}-{BACKEND 1}-{VERSION 1}.tar.gz
│   │   ├── ...
│   │   └── ${MODEL ...}-${ARCH ...}-{BACKEND ...}-{VERSION ...}.tar.gz
│   ├── models
│   │   ├── ${MODEL 1}
│   │   │   ├── ...
│   │   │   └── inference_engine_tvm_config.hpp
│   │   ├── ...
│   │   └── ${MODEL ...}
│   │       └── ...
│   └── user
│       ├── ${MODEL 1}
│       │   ├── deploy_graph.json
│       │   ├── deploy_lib.so
│       │   ├── deploy_param.params
│       │   └── inference_engine_tvm_config.hpp
│       ├── ...
│       └── ${MODEL ...}
│           └── ...
```

The `inference_engine_tvm_config.hpp` file needed for compilation by dependent packages is made available under "data/models/${MODEL_NAME}/inference_engine_tvm_config.hpp".
Dependent packages can use the cmake `add_dependencies` function with the name provided in the `DEPENDENCY` output parameter of `get_neural_network` to ensure this file is created before it gets used.

The other `deploy_*` files are installed to "models/${MODEL_NAME}/" under the `share` directory of the package.

The target version to be downloaded can be overwritten by setting the `MODELZOO_VERSION` cmake variable.

#### Assumptions / Known limits

If several packages make use of the same neural network, it will be downloaded once per package.

In case a requested artifact doesn't exist in the S3 bucket, the error message from ExternalProject is not explicit enough for the user to understand what went wrong.

In case the user manually sets `MODELZOO_VERSION` to "latest", the archive will not be re-downloaded when it gets updated in the S3 bucket (it is not a problem for tagged versions as they are not expected to be updated).

#### Inputs / Outputs

Inputs:

- `DOWNLOAD_ARTIFACTS` cmake variable; needs to be set to enable downloading the artifacts
- `MODELZOO_VERSION` cmake variable; can be used to overwrite the default target version of downloads

Outputs:

- `get_neural_network` cmake function; can be used to get a neural network compiled for a specific backend

In/Out:

- The `DEPENDENCY` argument of `get_neural_network` can be checked for the outcome of the function.
  It is an empty string when the neural network couldn't be made available.

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
