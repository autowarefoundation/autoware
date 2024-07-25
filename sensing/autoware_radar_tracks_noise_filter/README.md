# autoware_radar_tracks_noise_filter

This package contains a radar object filter module for `radar_msgs/msg/RadarTrack`.
This package can filter noise objects in RadarTracks.

## Algorithm

The core algorithm of this package is `RadarTrackCrossingNoiseFilterNode::isNoise()` function.
See the function and the parameters for details.

- Y-axis threshold

Radar can detect x-axis velocity as doppler velocity, but cannot detect y-axis velocity.
Some radar can estimate y-axis velocity inside the device, but it sometimes lack precision.
In y-axis threshold filter, if y-axis velocity of RadarTrack is more than `velocity_y_threshold`, it treats as noise objects.

## Input

| Name             | Type                           | Description         |
| ---------------- | ------------------------------ | ------------------- |
| `~/input/tracks` | radar_msgs/msg/RadarTracks.msg | 3D detected tracks. |

## Output

| Name                       | Type                           | Description      |
| -------------------------- | ------------------------------ | ---------------- |
| `~/output/noise_tracks`    | radar_msgs/msg/RadarTracks.msg | Noise objects    |
| `~/output/filtered_tracks` | radar_msgs/msg/RadarTracks.msg | Filtered objects |

## Parameters

| Name                   | Type   | Description                                                                                                                        | Default value |
| :--------------------- | :----- | :--------------------------------------------------------------------------------------------------------------------------------- | :------------ |
| `velocity_y_threshold` | double | Y-axis velocity threshold [m/s]. If y-axis velocity of RadarTrack is more than `velocity_y_threshold`, it treats as noise objects. | 7.0           |
