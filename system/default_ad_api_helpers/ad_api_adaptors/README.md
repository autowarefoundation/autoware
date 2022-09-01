# ad_api_adaptors

## initial_pose_adaptor

This node makes it easy to use the localization AD API from RViz.
When a initial pose topic is received, call the localization initialize API.
This node depends on fitting to map height service.

| Interface    | Local Name     | Global Name                       | Description                               |
| ------------ | -------------- | --------------------------------- | ----------------------------------------- |
| Subscription | initialpose    | /initialpose                      | The pose for localization initialization. |
| Client       | fit_map_height | /localization/util/fit_map_height | To fix initial pose to map height         |
| Client       | -              | /api/localization/initialize      | The localization initialize API.          |

## routing_adaptor

This node makes it easy to use the routing AD API from RViz.
When a goal pose topic is received, reset the waypoints and call the API.
When a waypoint pose topic is received, append it to the end of the waypoints to call the API.
The clear API is called automatically before setting the route.

| Interface    | Local Name       | Global Name                           | Description                 |
| ------------ | ---------------- | ------------------------------------- | --------------------------- |
| Subscription | ~/input/goal     | /planning/mission_planning/goal       | The goal pose of route.     |
| Subscription | ~/input/waypoint | /planning/mission_planning/checkpoint | The waypoint pose of route. |
| Client       | -                | /api/routing/clear_route              | The route clear API.        |
| Client       | -                | /api/routing/set_route_points         | The route points set API.   |
