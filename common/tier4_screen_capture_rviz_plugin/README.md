# tier4_screen_capture_rviz_plugin

## Purpose

This plugin captures the screen of rviz.

## Interface

| Name                         | Type                     | Description                        |
| ---------------------------- | ------------------------ | ---------------------------------- |
| `/debug/capture/video`       | `std_srvs::srv::Trigger` | Trigger to start screen capturing. |
| `/debug/capture/screen_shot` | `std_srvs::srv::Trigger` | Trigger to capture screen shot.    |

## Assumptions / Known limits

This is only for debug or analyze.
The `capture screen` button is still beta version which can slow frame rate.
set lower frame rate according to PC spec.

## Usage

1. Start rviz and select panels/Add new panel.
   ![select_panel](./images/select_panels.png)
