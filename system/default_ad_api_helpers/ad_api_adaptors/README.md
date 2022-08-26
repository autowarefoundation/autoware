# ad_api_adaptors

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
