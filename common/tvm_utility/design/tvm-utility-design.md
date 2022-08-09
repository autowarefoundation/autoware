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

### Inputs / Outputs / API

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

#### Outputs

- `autoware_check_neural_network` cmake macro to check if a specific network and backend combination exists

#### Backend

Dependent packages are expected to include `model_zoo.hpp` in order to get the TVM configuration structure of the targeted model/backend combination.
The backend used to do the inference can be specified by setting `NETWORKS_BACKEND` as a compile definition.
It defaults to `llvm`.

### Error detection and handling

`std::runtime_error` should be thrown whenever an error is encountered. It should be populated with an appropriate text
error description.

## Security considerations

Both the input and output are controlled by the same actor, so the following security concerns are out-of-scope:

- Spoofing
- Tampering

Leaking data to another actor would require a flaw in TVM or the host operating system that allows arbitrary memory to
be read, a significant security flaw in itself. This is also true for an external actor operating the pipeline early:
only the object that initiated the pipeline can run the methods to receive its output.

A Denial-of-Service attack could make the target hardware unusable for other pipelines but would require being able to
run code on the CPU, which would already allow a more severe Denial-of-Service attack.

No elevation of privilege is required for this package.

## Future extensions / Unimplemented parts

Future packages will use tvm_utility as part of the perception stack to run machine learning workloads.

## Related issues

<https://github.com/autowarefoundation/autoware/discussions/226>
