# constants
uint8 NONE=0
uint8 REQUESTING=1
uint8 PASSING=2
uint8 FINALIZING=3
uint8 FINALIZED=4

# fields
builtin_interfaces/Time stamp
string type
string id
tier4_v2x_msgs/KeyValue[] custom_tags # custom tags are set based on the 'type'
uint8 state
