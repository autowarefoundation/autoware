std_msgs/Header header
unique_identifier_msgs/UUID id
dummy_perception_publisher/InitialState initial_state
autoware_auto_perception_msgs/ObjectClassification classification
autoware_auto_perception_msgs/Shape shape
float32 max_velocity
float32 min_velocity

uint8 ADD=0
uint8 MODIFY=1
uint8 DELETE=2
uint8 DELETEALL=3
int32 action
