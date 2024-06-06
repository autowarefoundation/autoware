# Safety Check Utils

Safety check function checks if the given path will collide with a given target object.

## Purpose / Role

In the behavior path planner, certain modules (e.g., lane change) need to perform collision checks to ensure the safe navigation of the ego vehicle. These utility functions assist the user in conducting safety checks with other road participants.

### Assumptions

The safety check module is based on the following assumptions:

1. Users must provide the position, velocity, and shape of both the ego and target objects to the utility functions.
2. The yaw angle of each point in the predicted path of both the ego and target objects should point to the next point in the path.
3. The safety check module uses RSS distance to determine the safety of a potential collision with other objects.

### Limitations

Currently the yaw angle of each point of predicted paths of a target object does not point to the next point. Therefore, the safety check function might returns incorrect result for some edge case.

### Inner working / Algorithm

The flow of the safety check algorithm is described in the following explanations.

![safety_check_flow](../images/path_safety_checker/safety_check_flow.drawio.svg)

Here we explain each step of the algorithm flow.

#### 1. Get pose of the target object at a given time

For the first step, we obtain the pose of the target object at a given time. This can be done by interpolating the predicted path of the object.

#### 2. Check overlap

With the interpolated pose obtained in the step.1, we check if the object and ego vehicle overlaps at a given time. If they are overlapped each other, the given path is unsafe.

#### 3. Get front object

After the overlap check, it starts to perform the safety check for the broader range. In this step, it judges if ego or target object is in front of the other vehicle. We use arc length of the front point of each object along the given path to judge which one is in front of the other. In the following example, target object (red rectangle) is running in front of the ego vehicle (black rectangle).

![front_object](../images/path_safety_checker/front_object.drawio.svg)

#### 4. Calculate RSS distance

After we find which vehicle is running ahead of the other vehicle, we start to compute the RSS distance. With the reaction time $t_{reaction}$ and safety time margin $t_{margin}$, RSS distance can be described as:

$$
rss_{dist} = v_{rear} (t_{reaction} + t_{margin}) + \frac{v_{rear}^2}{2|a_{rear, decel}|} - \frac{v_{front}^2}{2|a_{front, decel|}}
$$

where $V_{front}$, $v_{rear}$ are front and rear vehicle velocity respectively and $a_{rear, front}$, $a_{rear, decel}$ are front and rear vehicle deceleration.

#### 5. Create extended ego and target object polygons

In this step, we compute extended ego and target object polygons. The extended polygons can be described as:

![extended_polygons](../images/path_safety_checker/extended_polygons.drawio.svg)

As the picture shows, we expand the rear object polygon. For the longitudinal side, we extend it with the RSS distance, and for the lateral side, we extend it by the lateral margin

#### 6. Check overlap

Similar to the previous step, we check the overlap of the extended rear object polygon and front object polygon. If they are overlapped each other, we regard it as the unsafe situation.
