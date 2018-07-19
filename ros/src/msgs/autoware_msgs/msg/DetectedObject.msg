std_msgs/Header                 header

uint32                          id
string                          label
float32                         score   #Score as defined by the detection, Optional
std_msgs/ColorRGBA              color   # Define this object specific color

################ 3D BB
string                          space_frame #3D Space coordinate frame of the object, required if pose and dimensions are defines
geometry_msgs/Pose              pose
geometry_msgs/Vector3           dimensions
geometry_msgs/Vector3           variance
geometry_msgs/Twist             velocity
geometry_msgs/Twist             acceleration

sensor_msgs/PointCloud2         pointcloud

geometry_msgs/PolygonStamped    convex_hull
autoware_msgs/LaneArray         candidate_trajectories

bool                            pose_reliable
bool                            velocity_reliable
bool                            acceleration_reliable

############### 2D Rect
string                          image_frame # Image coordinate Frame,        Required if x,y,w,h defined
int32                           x           # X coord in image space(pixel) of the initial point of the Rect
int32                           y           # Y coord in image space(pixel) of the initial point of the Rect
int32                           width       # Width of the Rect in pixels
int32                           height      # Height of the Rect in pixels
float32                         angle       # Angle [0 to 2*PI), allow rotated rects

sensor_msgs/Image               roi_image

# Behavior State of the Detected Object
uint8 FORWARD_STATE             = 0
uint8 STOPPING_STATE            = 1
uint8 BRANCH_LEFT_STATE         = 2
uint8 BRANCH_RIGHT_STATE        = 3
uint8 YIELDING_STATE            = 4
uint8 ACCELERATING_STATE        = 5
uint8 SLOWDOWN_STATE            = 6
uint8                           behavior_state

#
string[]                        user_defined_info