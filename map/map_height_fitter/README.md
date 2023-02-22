# map_height_fitter

This library fits the given point with the ground of the point cloud map.
The map loading operation is switched by the parameter `enable_partial_load` of the node specified by `map_loader_name`.
The node using this library must use multi thread executor.

| Interface    | Local Name         | Description                              |
| ------------ | ------------------ | ---------------------------------------- |
| Parameter    | map_loader_name    | The point cloud map loader name.         |
| Subscription | ~/pointcloud_map   | The topic name to load the whole map     |
| Client       | ~/partial_map_load | The service name to load the partial map |
