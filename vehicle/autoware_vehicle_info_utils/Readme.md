# Vehicle Info Util

## Purpose

This package is to get vehicle info parameters.

### Description

In [here](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/), you can check the vehicle dimensions with more detail.
The current format supports only the Ackermann model. This file defines the model assumed in autoware path planning, control, etc. and does not represent the exact physical model. If a model other than the Ackermann model is used, it is assumed that a vehicle interface will be designed to change the control output for the model.

### Versioning Policy

We have implemented a versioning system for the `vehicle_info.param.yaml` file to ensure clarity and consistency in file format across different versions of Autoware and its external applications. Please see [discussion](https://github.com/orgs/autowarefoundation/discussions/4050) for the details.

#### How to Operate

- The current file format is set as an unversioned base version (`version:` field is commented out).
- For the next update involving changes (such as additions, deletions, or modifications):
  - Uncomment and update the version line at the beginning of the file.
  - Initiate versioning by assigning a version number, starting from `0.1.0`. Follow the semantic versioning format (MAJOR.MINOR.PATCH).
  - Update this Readme.md too.
- For subsequent updates, continue incrementing the version number in accordance with the changes made.
  - Discuss how to increment version depending on the amount of changes made to the file.

```yaml
/**:
  ros__parameters:
    # version: 0.1.0 # Uncomment and update this line for future format changes.
    wheel_radius: 0.383
    ...
```

#### Why Versioning?

- Consistency Across Updates: Implementing version control will allow accurate tracking of changes over time and changes in vehicle information parameters.
- Clarity for External Applications: External applications that depend on `vehicle_info.param.yaml` need to reference the correct file version for optimal compatibility and functionality.
- Simplified Management for Customized Branches: Assigning versions directly to the `vehicle_info.param.yaml` file simplifies management compared to maintaining separate versions for multiple customized Autoware branches. This approach streamlines version tracking and reduces complexity.

### Scripts

#### Minimum turning radius

```sh
$ ros2 run autoware_vehicle_info_utils min_turning_radius_calculator.py
yaml path is /home/autoware/pilot-auto/install/autoware_vehicle_info_utils/share/autoware_vehicle_info_utils/config/vehicle_info.param.yaml
Minimum turning radius is 3.253042620027102 [m] for rear, 4.253220695862465 [m] for front.
```

You can designate yaml file with `-y` option as follows.

```sh
ros2 run autoware_vehicle_info_utils min_turning_radius_calculator.py -y <path-to-yaml>
```

## Assumptions / Known limits

TBD.
