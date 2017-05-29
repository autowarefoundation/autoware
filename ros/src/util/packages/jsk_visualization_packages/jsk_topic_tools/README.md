# jsk_topic_tools

This package includes several useful library for ROS software.

## C++ APIs
Please see [rosdoc](http://docs.ros.org/api/jsk_topic_tools/html/)

## How to generate document
```
$ rosdoc_lite `rospack find jsk_topic_tools` -o doc
```

## `pose_stamped_publisher.py`
Publish static `geometry_msgs/PoseStamped`.
```
Usage: pose_stamped_publisher.py x y z roll pitch yaw from_frame rate
```



## standalone_complexed_nodelet
### Why needed?
nodelet is a good framework to decrease overhead of network communication
between ROS nodes.

roscpp has a feature to communicate topics by passing reference pointers
if publisher and subscriber are in the same process.
By taking advantage of this feature, nodelet can communicate without network
overhead each other.

However, nodelet has a problem in system architecture of server-client model.
server is a manager and all the actual computation run in this process.
client just sends a request to manager to load some classes in the manager process
by dynamic loading.

The issue occurs in bond between server and client. If server is killded, client should be killed
as well. If a client is killed, the class which is requested by the client loaded in server process should
be unloded. nodelet implements this functionality by [bond](https://github.com/ros/bond_core) package.
server publishes heatbeat topic in fixed rate and client subscribes the topic. If client detects stoll of
the topic, client regards manager is dead and kill itself and request unloading to the manager.

This mechanism works well if nodelet system is relatively small but if we run a lot of nodelet, it does not work
because communication via ROS topics is not reliable and somtimes it has a lot of latency.

This "unreliable bond mechanism" issue occurs a lot and `standalone_complexed_nodelet` can solve it.
`standalone_complexed_nodelet` is a nodelet manager but it does not require client to send request.
`standalone_complexed_nodelet` knows nodelet classes via ros parameter instead of server-client model.
Limitation of `standalone_complexed_nodelet` is that it does support dynamic loading/unloading of nodelet
classes but we know we do not need the feature from our experiences.

We recommend to use `standalone_complexed_nodelet` instead of normal `nodelet`.

### APIs
`standalone_complexed_nodelet` read definition of nodelet classes from ros parameter. The parameter are `~nodelets`, `~nodelets_0`, ..., `~nodelets_99`.
You can extend the suffix numbers by `~candidate_num` parameter. default is 100.
These multiple parameters (`~nodelets_0`, `nodelets_1`, ...) are useful
to include multiple launch files and run one `standalone_complexed_nodelet`.

nodelet definition should follow next format:
```yaml
nodelets:
  - name: <name of nodelet, required>
    type: <type of nodelet, required>
    if: <the defnition is enabled only if this field is true, optional>
    unless: <the defnition is enabled only if this field is false, optional>
    remappings: <field to lead remapping definition, optional>
      - from: <ros name to be remapped>
        to: <ros name to remap to>
      - from:
        to:
        ...
  - name:
    type:
    remappings:
    ...
```
