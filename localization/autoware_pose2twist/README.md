# autoware_pose2twist

## Purpose

This `autoware_pose2twist` calculates the velocity from the input pose history. In addition to the computed twist, this node outputs the linear-x and angular-z components as a float message to simplify debugging.

The `twist.linear.x` is calculated as `sqrt(dx * dx + dy * dy + dz * dz) / dt`, and the values in the `y` and `z` fields are zero.
The `twist.angular` is calculated as `relative_rotation_vector / dt` for each field.

## Inputs / Outputs

### Input

| Name | Type                            | Description                                       |
| ---- | ------------------------------- | ------------------------------------------------- |
| pose | geometry_msgs::msg::PoseStamped | pose source to used for the velocity calculation. |

### Output

| Name      | Type                                  | Description                                   |
| --------- | ------------------------------------- | --------------------------------------------- |
| twist     | geometry_msgs::msg::TwistStamped      | twist calculated from the input pose history. |
| linear_x  | tier4_debug_msgs::msg::Float32Stamped | linear-x field of the output twist.           |
| angular_z | tier4_debug_msgs::msg::Float32Stamped | angular-z field of the output twist.          |

## Parameters

none.

## Assumptions / Known limits

none.
