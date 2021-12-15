vehicle_constants_manager {#vehicle-constants-manager-package-design}
===========

This is the design document for the `vehicle_constants_manager` package.

# Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This library provides a struct for holding vehicle specific constants. It also
provides a helper method to declare vehicle specific constants which have
already been passed into a node and provide a `VehicleConstants` object.

# Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
Provided `VehicleConstants` struct holds vehicle parameters. Its parameters can
be split in 2 categories:
(The detailed descriptions and units of the variables is in the
`vehicle_constants_manager.hpp` file.)

- Primary Constants
    - wheel_radius
    - wheel_width
    - wheel_base
    - wheel_tread
    - overhang_front
    - overhang_rear
    - overhang_left
    - overhang_right
    - vehicle_height
    - cg_to_rear
    - tire_cornering_stiffness_front_n_per_deg
    - tire_cornering_stiffness_rear_n_per_deg
    - mass_vehicle
    - inertia_yaw_kg_m_2
- Derived Constants
    - cg_to_front
    - vehicle_length
    - vehicle_width
    - offset_longitudinal_min
    - offset_longitudinal_max
    - offset_lateral_min
    - offset_lateral_max
    - offset_height_min
    - offset_height_max

The `VehicleConstants` constructor is initialized with the primary parameters.

The library also provides a `declare_and_get_vehicle_constants` method. Using
this method, the user can declare vehicle parameters that are already passed
into the node and obtain a `VehicleConstants` object.

## Assumptions / Known limits

<!-- Required -->

This library assumes the vehicle is defined with Ackermann steering geometry.

`declare_and_get_vehicle_constants` method requires the passed node to have following parameters overridden:

(Pay attention to the `vehicle` namespace)
```yaml
vehicle:
  wheel_radius:
  wheel_width:
  wheel_base:
  wheel_tread:
  overhang_front:
  overhang_rear:
  overhang_left:
  overhang_right:
  vehicle_height:
  cg_to_rear:
  tire_cornering_stiffness_front_n_per_deg:
  tire_cornering_stiffness_rear_n_per_deg:
  mass_vehicle:
  inertia_yaw_kg_m_2:
```


## Inputs / Outputs / API {#vehicle-constants-manager-package-design-inputs}

<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

The constructor of `VehicleConstants` takes the primary vehicle constants and
generates the derived parameters.

`declare_and_get_vehicle_constants` method takes a `rclcpp::Node` object. And 
returns a `VehicleConstants` object if it succeeds.

Example usage:
```cpp
// In the constructor of a node which received primary vehicle parameters from a
// .yaml file or run args.
auto vehicle_constants = declare_and_get_vehicle_constants(*this);
```

## Inner-workings / Algorithms

<!-- If applicable -->
Not Available.

## Error detection and handling

<!-- Required -->

The `VehicleConstants` struct performs some sanity checks upon construction.

It will throw `std::runtime_error` in case certain parameters are negative or
cg_to_rear is larger than wheel_base (to ensure center of gravity is within
front and rear axles.)

# Security considerations

<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->
To Be Determined.

# References / External links

<!-- Optional -->
Not Available.

# Future extensions / Unimplemented parts

<!-- Optional -->
Not Available.

# Related issues

<!-- Required -->
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1294
