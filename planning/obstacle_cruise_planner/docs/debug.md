# Debug

## Debug visualization

### Detection area

Green polygons which is a detection area is visualized by `detection_polygons` in the `~/debug/marker` topic.

![detection_area](./image/detection_area.png)

### Collision point

Red point which is a collision point with obstacle is visualized by `collision_points` in the `~/debug/marker` topic.

![collision_point](./image/collision_point.png)

### Obstacle for cruise

Yellow sphere which is a obstacle for cruise is visualized by `obstacles_to_cruise` in the `~/debug/marker` topic.

![obstacle_to_cruise](./image/obstacle_to_cruise.png)

### Obstacle for stop

Red sphere which is a obstacle for stop is visualized by `obstacles_to_stop` in the `~/debug/marker` topic.

![obstacle_to_stop](./image/obstacle_to_stop.png)

<!-- ### Obstacle ignored to cruise or stop intentionally -->

<!-- Green sphere which is a obstacle ignored intentionally to cruise or stop is visualized by `intentionally_ignored_obstacles` in the `~/debug/marker` topic. -->

<!-- ![intentionally_ignored_obstacle](./image/intentionally_ignored_obstacle.png) -->

### Obstacle cruise wall

Yellow wall which means a safe distance to cruise if the ego's front meets the wall is visualized in the `~/debug/cruise_wall_marker` topic.

![obstacle_to_cruise](./image/obstacle_to_cruise.png)

### Obstacle stop wall

Red wall which means a safe distance to stop if the ego's front meets the wall is visualized in the `~/debug/stop_wall_marker` topic.

![obstacle_to_stop](./image/obstacle_to_stop.png)
