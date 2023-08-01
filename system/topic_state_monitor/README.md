# topic_state_monitor

## Purpose

This node monitors input topic for abnormalities such as timeout and low frequency.
The result of topic status is published as diagnostics.

## Inner-workings / Algorithms

The types of topic status and corresponding diagnostic status are following.

| Topic status  | Diagnostic status | Description                                          |
| ------------- | ----------------- | ---------------------------------------------------- |
| `OK`          | OK                | The topic has no abnormalities                       |
| `NotReceived` | ERROR             | The topic has not been received yet                  |
| `WarnRate`    | WARN              | The frequency of the topic is dropped                |
| `ErrorRate`   | ERROR             | The frequency of the topic is significantly dropped  |
| `Timeout`     | ERROR             | The topic subscription is stopped for a certain time |

## Inputs / Outputs

### Input

| Name     | Type     | Description                       |
| -------- | -------- | --------------------------------- |
| any name | any type | Subscribe target topic to monitor |

### Output

| Name           | Type                              | Description         |
| -------------- | --------------------------------- | ------------------- |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics outputs |

## Parameters

### Node Parameters

| Name              | Type   | Default Value | Description                                                   |
| ----------------- | ------ | ------------- | ------------------------------------------------------------- |
| `topic`           | string | -             | Name of target topic                                          |
| `topic_type`      | string | -             | Type of target topic (used if the topic is not transform)     |
| `frame_id`        | string | -             | Frame ID of transform parent (used if the topic is transform) |
| `child_frame_id`  | string | -             | Frame ID of transform child (used if the topic is transform)  |
| `transient_local` | bool   | false         | QoS policy of topic subscription (Transient Local/Volatile)   |
| `best_effort`     | bool   | false         | QoS policy of topic subscription (Best Effort/Reliable)       |
| `diag_name`       | string | -             | Name used for the diagnostics to publish                      |
| `update_rate`     | double | 10.0          | Timer callback period [Hz]                                    |

### Core Parameters

| Name          | Type   | Default Value | Description                                                                                          |
| ------------- | ------ | ------------- | ---------------------------------------------------------------------------------------------------- |
| `warn_rate`   | double | 0.5           | If the topic rate is lower than this value, the topic status becomes `WarnRate`                      |
| `error_rate`  | double | 0.1           | If the topic rate is lower than this value, the topic status becomes `ErrorRate`                     |
| `timeout`     | double | 1.0           | If the topic subscription is stopped for more than this time [s], the topic status becomes `Timeout` |
| `window_size` | int    | 10            | Window size of target topic for calculating frequency                                                |

## Assumptions / Known limits

TBD.
