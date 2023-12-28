# Landmark Based Localizer

This directory contains packages for landmark-based localization.

Landmarks are, for example

- AR tags detected by camera
- Boards characterized by intensity detected by LiDAR

etc.

Since these landmarks are easy to detect and estimate pose, the ego pose can be calculated from the pose of the detected landmark if the pose of the landmark is written on the map in advance.

Currently, landmarks are assumed to be flat.

The following figure shows the principle of localization in the case of `ar_tag_based_localizer`.

![principle](./doc_image/principle.png)

This calculated ego pose is passed to the EKF, where it is fused with the twist information and used to estimate a more accurate ego pose.

## Node diagram

![node diagram](./doc_image/node_diagram.drawio.svg)

### `landmark_manager`

The definitions of the landmarks written to the map are introduced in the next section. See `Map Specifications`.

The `landmark_manager` is a utility package to load landmarks from the map.

- Translation : The center of the four vertices of the landmark
- Rotation : Let the vertex numbers be 1, 2, 3, 4 counterclockwise as shown in the next section. Direction is defined as the cross product of the vector from 1 to 2 and the vector from 2 to 3.

Users can define landmarks as Lanelet2 4-vertex polygons.
In this case, it is possible to define an arrangement in which the four vertices cannot be considered to be on the same plane. The direction of the landmark in that case is difficult to calculate.
So, if the 4 vertices are considered as forming a tetrahedron and its volume exceeds the `volume_threshold` parameter, the landmark will not publish tf_static.

### Landmark based localizer packages

- ar_tag_based_localizer
- etc.

## Map specifications

See <https://github.com/autowarefoundation/autoware_common/blob/main/tmp/lanelet2_extension/docs/lanelet2_format_extension.md#localization-landmarks>

## About `consider_orientation`

The `calculate_new_self_pose` function in the `LandmarkManager` class includes a boolean argument named `consider_orientation`. This argument determines the method used to calculate the new self pose based on detected and mapped landmarks. The following image illustrates the difference between the two methods.

![consider_orientation_figure](./doc_image/consider_orientation.drawio.svg)

### `consider_orientation = true`

In this mode, the new self pose is calculated so that the relative Pose of the "landmark detected from the current self pose" is equal to the relative Pose of the "landmark mapped from the new self pose".
This method can correct for orientation, but is strongly affected by the orientation error of the landmark detection.

### `consider_orientation = false`

In this mode, the new self pose is calculated so that only the relative position is correct for x, y, and z.

This method can not correct for orientation, but it is not affected by the orientation error of the landmark detection.
