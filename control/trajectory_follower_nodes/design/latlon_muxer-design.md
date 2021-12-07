Lateral/Longitudinal Control Muxer {#latlon-muxer-design}
=============================================

# Purpose

When using controllers that independently compute lateral and longitudinal commands,
this node combines the resulting messages into a single control command message.

# Design

Inputs.
- `AckermannLateralCommand`: lateral command.
- `LongitudinalCommand`: longitudinal command.

Output.
- `AckermannControlCommand`: message containing both lateral and longitudinal commands.

Parameter.
- `timeout_thr_sec`: duration in second after which input messages are discarded.

Each time the node receives an input message it publishes an `AckermannControlCommand`
if the following two conditions are met.
1. Both inputs have been received.
2. The last received input messages are not older than defined by `timeout_thr_sec`.

# Implementation Details

Callbacks `latCtrlCmdCallback` and `lonCtrlCmdCallback` are defined for each input message.
Upon reception, the message is stored and function `publishCmd` is called.

Function `publishCmd` first checks that both messages have been received
and that the stored messages are not older than the timeout.
If both conditions are true, the combined control message is built and published.

`checkTimeout` is used to check that the stored messages are not too old
by comparing the timeout parameter `timeout_thr_sec`
with the difference between `rclcpp::Time stamp` and the current time `now()`.
