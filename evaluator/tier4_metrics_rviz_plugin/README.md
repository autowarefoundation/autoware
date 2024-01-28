# tier4_metrics_rviz_plugin

## Purpose

This plugin panel to visualize `planning_evaluator` output.

## Inputs / Outputs

| Name                                     | Type                                    | Description                           |
| ---------------------------------------- | --------------------------------------- | ------------------------------------- |
| `/diagnostic/planning_evaluator/metrics` | `diagnostic_msgs::msg::DiagnosticArray` | Subscribe `planning_evaluator` output |

## HowToUse

1. Start rviz and select panels/Add new panel.
2. Select MetricsVisualizePanel and press OK.
