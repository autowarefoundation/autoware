# Routing API

## Overview

Unify the route setting method to the service. This API supports two waypoint formats, poses and lanelet segments. The goal and checkpoint topics from rviz is only subscribed to by adapter node and converted to API call. This API call is forwarded to the mission planner node so it can centralize the state of routing. For other nodes that require route, mission planner node publishes as `/planning/mission_planning/route`. See the [autoware-documentation](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/routing/) for AD API specifications.

![routing-architecture](images/routing.drawio.svg)
