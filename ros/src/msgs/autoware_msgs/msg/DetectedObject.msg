std_msgs/Header header

uint32 id
string label

geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions
geometry_msgs/Vector3 variance
geometry_msgs/Twist velocity

geometry_msgs/PolygonStamped convex_hull
autoware_msgs/LaneArray candidate_trajectories

bool pose_reliable
bool velocity_reliable

# Behavior State of the Detected Object
# FORWARD_STATE			= 0
# STOPPING_STATE 		= 1
# BRANCH_LEFT_STATE		= 2
# BRANCH_RIGHT_STATE	= 3 
# YIELDING_STATE		= 4
# ACCELERATING_STATE	= 5
# SLOWDOWN_STATE 		= 6

uint32 behavior_state

# Indicator information
# INDICATOR_LEFT 0
# INDICATOR_RIGHT 1
# INDICATOR_BOTH 2
# INDICATOR_NONE 3
uint32 indicator_state