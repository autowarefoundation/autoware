# autoware_auto_tf2

This is the design document for the `autoware_auto_tf2` package.

## Purpose / Use cases

In general, users of ROS rely on tf (and its successor, tf2) for publishing and utilizing coordinate
frame transforms. This is true even to the extent that the tf2 contains the packages
`tf2_geometry_msgs` and `tf2_sensor_msgs` which allow for easy conversion to and from the message
types defined in `geometry_msgs` and `sensor_msgs`, respectively. However, AutowareAuto contains
some specialized message types which are not transformable between frames using the ROS 2 library.
The `autoware_auto_tf2` package aims to provide developers with tools to transform applicable
`autoware_auto_msgs` types. In addition to this, this package also provides transform tools for
messages types in `geometry_msgs` missing in `tf2_geometry_msgs`.

## Design

While writing `tf2_some_msgs` or contributing to `tf2_geometry_msgs`, compatibility and design
intent was ensured with the following files in the existing tf2 framework:

- `tf2/convert.h`
- `tf2_ros/buffer_interface.h`

For example:

```cpp
void tf2::convert( const A & a,B & b)
```

The method `tf2::convert` is dependent on the following:

```cpp
template<typename A, typename B>
  B tf2::toMsg(const A& a);
template<typename A, typename B>
  void tf2::fromMsg(const A&, B& b);

// New way to transform instead of using tf2::doTransform() directly
tf2_ros::BufferInterface::transform(...)
```

Which, in turn, is dependent on the following:

```cpp
void tf2::convert( const A & a,B & b)
const std::string& tf2::getFrameId(const T& t)
const ros::Time& tf2::getTimestamp(const T& t);
```

## Current Implementation of tf2_geometry_msgs

In both ROS 1 and ROS 2 stamped msgs like `Vector3Stamped`, `QuaternionStamped` have associated
functions like:

- `getTimestamp`
- `getFrameId`
- `doTransform`
- `toMsg`
- `fromMsg`

In ROS 1, to support `tf2::convert` and need in `doTransform` of the stamped data, non-stamped
underlying data like `Vector3`, `Point`, have implementations of the following functions:

- `toMsg`
- `fromMsg`

In ROS 2, much of the `doTransform` method is not using `toMsg` and `fromMsg` as data types from tf2
are not used. Instead `doTransform` is done using `KDL`, thus functions relating to underlying data
were not added; such as `Vector3`, `Point`, or ported in this commit ros/geometry2/commit/6f2a82.
The non-stamped data with `toMsg` and `fromMsg` are `Quaternion`, `Transform`. `Pose` has the
modified `toMsg` and not used by `PoseStamped`.

## Plan for autoware_auto_perception_msgs::msg::BoundingBoxArray

The initial rough plan was to implement some of the common tf2 functions like `toMsg`, `fromMsg`,
and `doTransform`, as needed for all the underlying data types in `BoundingBoxArray`. Examples
of the data types include: `BoundingBox`, `Quaternion32`, and `Point32`. In addition, the
implementation should be done such that upstream contributions could also be made to `geometry_msgs`.

## Assumptions / Known limits

Due to conflicts in a function signatures, the predefined template of `convert.h`/
`transform_functions.h` is not followed and compatibility with `tf2::convert(..)` is broken and
`toMsg` is written differently.

```cpp
// Old style
geometry_msgs::Vector3 toMsg(const tf2::Vector3& in)
geometry_msgs::Point& toMsg(const tf2::Vector3& in)

// New style
geometry_msgs::Point& toMsg(const tf2::Vector3& in, geometry_msgs::Point& out)
```

## Inputs / Outputs / API

<!-- Required -->

The library provides API `doTransform` for the following data-types that are either not available
in `tf2_geometry_msgs` or the messages types are part of `autoware_auto_msgs` and are therefore
custom and not inherently supported by any of the tf2 libraries. The following APIs are provided
for the following data types:

- `Point32`

```cpp
inline void doTransform(
  const geometry_msgs::msg::Point32 & t_in,
  geometry_msgs::msg::Point32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
```

- `Quaternion32` (`autoware_auto_msgs`)

```cpp
inline void doTransform(
  const autoware_auto_geometry_msgs::msg::Quaternion32 & t_in,
  autoware_auto_geometry_msgs::msg::Quaternion32 & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
```

- `BoundingBox` (`autoware_auto_msgs`)

```cpp
inline void doTransform(
  const BoundingBox & t_in, BoundingBox & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
```

- `BoundingBoxArray`

```cpp
inline void doTransform(
  const BoundingBoxArray & t_in,
  BoundingBoxArray & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
```

In addition, the following helper methods are also added:

- `BoundingBoxArray`

```cpp
inline tf2::TimePoint getTimestamp(const BoundingBoxArray & t)

inline std::string getFrameId(const BoundingBoxArray & t)
```

<!-- ## Inner-workings / Algorithms -->
<!-- If applicable -->

<!-- ## Error detection and handling -->
<!-- Required -->

<!-- # Security considerations -->
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

<!-- # References / External links -->
<!-- Optional -->

## Future extensions / Unimplemented parts

## Challenges

- `tf2_geometry_msgs` does not implement `doTransform` for any non-stamped data types, but it is
  possible with the same function template. It is needed when transforming sub-data, with main data
  that does have a stamp and can call doTransform on the sub-data with the same transform. Is this a useful upstream contribution?
- `tf2_geometry_msgs` does not have `Point`, `Point32`, does not seem it needs one, also the
  implementation of non-standard `toMsg` would not help the convert.
- `BoundingBox` uses 32-bit float like `Quaternion32` and `Point32` to save space, as they are used
  repeatedly in `BoundingBoxArray`. While transforming is it better to convert to 64-bit `Quaternion`,
  `Point`, or `PoseStamped`, to re-use existing implementation of `doTransform`, or does it need to be
  implemented? It may not be simple to template.

<!-- # Related issues -->
<!-- Required -->
