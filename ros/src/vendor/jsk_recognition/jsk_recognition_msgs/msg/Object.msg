# object instant info (ex. oreo_cookie)
int32 id  # object id
string name  # object name

# object class info (ex. snack)
int32 class_id
string class_name

string[] image_resources  # image urls
string mesh_resource  # mesh file url

float32 weight  # weight [kg]
geometry_msgs/Vector3 dimensions  # bounding box [m]
