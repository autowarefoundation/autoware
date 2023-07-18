## No Drivable Lane

### Role

This module plans the velocity of the related part of the path in case there is a no drivable lane referring to it.

A no drivable lane is a lanelet or more that are out of operation design domain (ODD), i.e., the vehicle **must not** drive autonomously in this lanelet.  
A lanelet can be no drivable (out of ODD) due to many reasons, either technical limitations of the SW and/or HW, business requirements, safety considerations, .... etc, or even a combination of those.

Some examples of No Drivable Lanes

- Closed road intentionally, due to construction work for example
- Underpass road that goes under a railway, for safety reasons
- Road with slope/inclination that the vehicle is not be able to drive autonomously due to technical limitations. And lots of other examples.

![no-drivable-lane-design.svg](./docs/no_drivable_lane_design.svg)

A lanelet becomes invalid by adding a new tag under the relevant lanelet in the map file `<tag k="no_drivable_lane" v="yes"/>`.

The target of this module is to stop the vehicle before entering the no drivable lane (with configurable stop margin) or keep the vehicle stationary if autonomous mode started inside a no drivable lane. Then ask the human driver to take the responsibility of the driving task (Takeover Request / Request to Intervene)

### Activation Timing

This function is activated when the lane id of the target path has an no drivable lane label (i.e. the `no_drivable_lane` attribute is `yes`).

### Module Parameters

| Parameter          | Type   | Description                                          |
| ------------------ | ------ | ---------------------------------------------------- |
| `stop_margin`      | double | [m] margin for ego vehicle to stop before speed_bump |
| `print_debug_info` | bool   | whether debug info will be printed or not            |

### Inner-workings / Algorithms

- Get no_drivable_lane attribute on the path from lanelet2 map
- The no drivable lane state machine starts in `INIT` state
- Get the intersection points between path and no drivable lane polygon
- Assign the state to `APPROACHING` toward a no drivable lane if:
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the no drivable lane polygon is more than the `stop_margin`
- Assign the state to `INSIDE_NO_DRIVABLE_LANE` if:
  - the first point of the ego path is inside the no drivable lane polygon, or
  - the distance from front of the ego vehicle till the first intersection point between the ego path and the no drivable lane polygon is less than the `stop_margin`
- Assign the state to `STOPPED` when the vehicle is completely stopped

![no_drivable_lane_scenarios.svg](./docs/no_drivable_lane_scenarios.svg)

### Future Work

- As [Request to Intervene API](https://github.com/autowarefoundation/autoware/issues/3487) is not implemented yet, this will be handled to notify the driver to takeover the driving task responsibility after the vehicle stops due to `no_drivable_lane`
- Handle the case when the vehicle stops before a no drivable lane but part of its footprint intersects with the no drivable lane polygon.
