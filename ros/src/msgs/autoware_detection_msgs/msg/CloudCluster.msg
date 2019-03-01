std_msgs/Header header

uint32 id
string label
float64 score

sensor_msgs/PointCloud2 cloud

geometry_msgs/PointStamped min_point
geometry_msgs/PointStamped max_point
geometry_msgs/PointStamped avg_point
geometry_msgs/PointStamped centroid_point

float64 estimated_angle

geometry_msgs/Vector3 dimensions
geometry_msgs/Vector3 eigen_values
geometry_msgs/Vector3[] eigen_vectors

#Array of 33 floats containing the FPFH descriptor
std_msgs/Float32MultiArray fpfh_descriptor 

jsk_recognition_msgs/BoundingBox bounding_box
geometry_msgs/PolygonStamped convex_hull

# Indicator information
# INDICATOR_LEFT 0
# INDICATOR_RIGHT 1
# INDICATOR_BOTH 2
# INDICATOR_NONE 3
uint32 indicator_state