# Spatial Hash

The spatial hash is a data structure designed for efficient fixed-radius near-neighbor queries in
low dimensions.

The fixed-radius near-neighbors problem is defined as follows:

`For point p, find all points p' s.t. d(p, p') < r`

Where in this case `d(p, p')` is euclidean distance, and `r` is the fixed
radius.

For `n` points with an average of `k` neighbors each, this data structure can
perform `m` near-neighbor queries (to generate lists of near-neighbors for `m`
different points) in `O(mk)` time.

By contrast, using a k-d tree for successive nearest-neighbor queries results in
a running time of `O(m log n)`.

The spatial hash works as follows:

- Each point is assigned to a bin in the predefined bounding area defined by
  `x_min/x_max` and `y_min/y_max`
- This can be done by converting x and y position into x and y index
  respectively
  - For example with the bin containing `x_min` and `y_min` as index `(0, 0)`
  - The two (or more) indices can then be converted into a single index
- Once every point of interest has been inserted into the hash, near-neighbor
  queries can begin:
  - The bin of the reference point is first computed
  - For each point in each adjacent bin, perform an explicit distance computation
    between said point and the reference point. If the distance is below the given
    radius, said point is considered to be a near-neighbor

Under the hood, an `std::unordered_multimap` is used, where the key is a bin/voxel index.
The bin size was computed to be the same as the lookup distance.

<!-- cspell:ignore CRTP -->

In addition, this data structure can support 2D or 3D queries. This is determined during
configuration, and baked into the data structure via the configuration class. The purpose of
this was to avoid if statements in tight loops. The configuration class specializations themselves
use CRTP (Curiously Recurring Template Patterns) to do "static polymorphism", and avoid
a dispatching call.

## Performance characterization

### Time

Insertion is `O(n)` because lookup time for the underlying hashmap is `O(n)` for
hashmaps. In practice, lookup time for hashmaps and thus insertion time should
be `O(1)`.

Removing a point is `O(1)` because the current API only supports removal via
direct reference to a node.

Finding `k` near-neighbors is worst case `O(n)` in the case of an adversarial
example, but in practice `O(k)`.

### Space

The module consists of the following components:

- The internal hashmap is `O(n + n + A * n)`, where `A` is an arbitrary
  constant (load factor)
- The other components of the spatial hash are `O(n + n)`

This results in `O(n)` space complexity.

## States

The spatial hash's state is dictated by the status of the underlying unordered_multimap.

The data structure is wholly configured by a
[config](@ref autoware::common::geometry::spatial_hash::Config) class. The constructor
of the class determines in the data structure accepts strictly 2D or strictly 3D queries.

## Inputs

The primary method of introducing data into the data structure is via the
[insert](@ref autoware::common::geometry::spatial_hash::SpatialHashBase::insert) method.

## Outputs

The primary method of retrieving data from the data structure is via the
[near](@ref autoware::common::geometry::spatial_hash::SpatialHash<PointT, Config2d>::near)\(2D
configuration\)
or [near](@ref autoware::common::geometry::spatial_hash::SpatialHash<PointT, Config3d>::near)
\(3D configuration\) method.

The whole data structure can also be traversed using standard constant iterators.

## Future Work

- Performance tuning and optimization

## Related issues

- #28: Port to autoware.Auto
