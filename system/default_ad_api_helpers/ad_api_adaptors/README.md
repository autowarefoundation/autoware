# ad_api_adaptors

## initial_pose_adaptor

This node makes it easy to use the localization AD API from RViz.
When a initial pose topic is received, call the localization initialize API.
This node depends on the map height fitter library.
[See here for more details.](../../../map/map_height_fitter/README.md)

| Interface    | Local Name  | Global Name                  | Description                               |
| ------------ | ----------- | ---------------------------- | ----------------------------------------- |
| Subscription | initialpose | /initialpose                 | The pose for localization initialization. |
| Client       | -           | /api/localization/initialize | The localization initialize API.          |

## routing_adaptor

This node makes it easy to use the routing AD API from RViz.
When a goal pose topic is received, reset the waypoints and call the API.
When a waypoint pose topic is received, append it to the end of the waypoints to call the API.
The clear API is called automatically before setting the route.

| Interface    | Local Name         | Global Name                           | Description                                        |
| ------------ | ------------------ | ------------------------------------- | -------------------------------------------------- |
| Subscription | -                  | /api/routing/state                    | The state of the routing API.                      |
| Subscription | ~/input/fixed_goal | /planning/mission_planning/goal       | The goal pose of route. Disable goal modification. |
| Subscription | ~/input/rough_goal | /rviz/routing/rough_goal              | The goal pose of route. Enable goal modification.  |
| Subscription | ~/input/reroute    | /rviz/routing/reroute                 | The goal pose of reroute.                          |
| Subscription | ~/input/waypoint   | /planning/mission_planning/checkpoint | The waypoint pose of route.                        |
| Client       | -                  | /api/routing/clear_route              | The route clear API.                               |
| Client       | -                  | /api/routing/set_route_points         | The route points set API.                          |
| Client       | -                  | /api/routing/change_route_points      | The route points change API.                       |
