# Vehicle Info Util

## Purpose

This package is to get vehicle info parameters.

### Description

In [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/), you can check the vehicle dimensions with more detail.

### Scripts

#### Minimum turning radius

```sh
$ ros2 run vehicle_info_util min_turning_radius_calculator.py
yaml path is /home/autoware/pilot-auto/install/vehicle_info_util/share/vehicle_info_util/config/vehicle_info.param.yaml
Minimum turning radius is 3.253042620027102 [m] for rear, 4.253220695862465 [m] for front.
```

You can designate yaml file with `-y` option as follows.

```sh
ros2 run vehicle_info_util min_turning_radius_calculator.py -y <path-to-yaml>
```

## Assumptions / Known limits

TBD.
