# default_ad_api

## Features

This package is a default implementation AD API.

- [autoware state (backward compatibility)](document/autoware-state.md)
- [fail-safe](document/fail-safe.md)
- [interface](document/interface.md)
- [localization](document/localization.md)
- [motion](document/motion.md)
- [operation mode](document/operation-mode.md)
- [routing](document/routing.md)

## Web server script

This is a sample to call API using HTTP.

## Guide message script

This is a debug script to check the conditions for transition to autonomous mode.

```bash
$ ros2 run default_ad_api guide.py

The vehicle pose is not estimated. Please set an initial pose or check GNSS.
The route is not set. Please set a goal pose.
The topic rate error is detected. Please check [control,planning] components.
The vehicle is ready. Please change the operation mode to autonomous.
The vehicle is driving autonomously.
The vehicle has reached the goal of the route. Please reset a route.
```
