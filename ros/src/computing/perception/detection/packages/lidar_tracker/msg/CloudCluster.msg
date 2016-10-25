std_msgs/Header header

uint32 id
string label

sensor_msgs/PointCloud2 cloud

geometry_msgs/Point min_point
geometry_msgs/Point max_point
geometry_msgs/Point avg_point
geometry_msgs/Point centroid_point

float64 estimated_angle

geometry_msgs/Vector3 dimensions
geometry_msgs/Vector3 eigen_values
geometry_msgs/Vector3[] eigen_vectors

jsk_recognition_msgs/BoundingBox bounding_box