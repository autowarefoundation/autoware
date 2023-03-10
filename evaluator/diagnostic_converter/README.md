# Planning Evaluator

## Purpose

This package provides a node to convert `diagnostic_msgs::msg::DiagnosticArray` messages
into `tier4_simulation_msgs::msg::UserDefinedValue` messages.

## Inner-workings / Algorithms

The node subscribes to all topics listed in the parameters and assumes they publish
`DiagnosticArray` messages.
Each time such message is received,
it is converted into as many `UserDefinedValue` messages as the number of `KeyValue` objects.
The format of the output topic is detailed in the _output_ section.

## Inputs / Outputs

### Inputs

The node listens to `DiagnosticArray` messages on the topics specified in the parameters.

### Outputs

The node outputs `UserDefinedValue` messages that are converted from the received `DiagnosticArray`.

The name of the output topics are generated from the corresponding input topic, the name of the diagnostic status, and the key of the diagnostic.
For example, we might listen to topic `/diagnostic_topic` and receive a `DiagnosticArray` with 2 status:

- Status with `name: "x"`.
  - Key: `a`.
  - Key: `b`.
- Status with `name: "y"`.
  - Key: `a`.
  - Key: `c`.

The resulting topics to publish the `UserDefinedValue` are as follows:

- `/metrics_x_a`.
- `/metrics_x_b`.
- `/metrics_y_a`.
- `/metrics_y_c`.

## Parameters

| Name                | Type             | Description                                                   |
| :------------------ | :--------------- | :------------------------------------------------------------ |
| `diagnostic_topics` | list of `string` | list of DiagnosticArray topics to convert to UserDefinedValue |

## Assumptions / Known limits

Values in the `KeyValue` objects of a `DiagnosticStatus` are assumed to be of type `double`.

## Future extensions / Unimplemented parts
