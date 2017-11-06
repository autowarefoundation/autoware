#define MQTT_NODE_NAME "mqtt_socket"
#define MQTT_CONFIG_FILE_NAME "/mqtt_config.yml"

// MODE
#define AUTO_MODE       1
#define REMOTE_MODE     2
#define NORMAL_MODE     0
#define EMERGENCY_MODE  1

// REMOTE CMD
static int vehicle_id;
static int accel_max_val;
static int brake_max_val;
static float steer_max_val;
static float linear_x_max_val;

// CANINFO
static float caninfo_downsample;

// GEAR
int gear_d;
int gear_n;
int gear_r;
int gear_p;

// MQTT
static struct mosquitto *mqtt_client = NULL;
static string mqtt_client_id;
static string mqtt_address;
static int mqtt_port;
static string mqtt_topic;
static int mqtt_qos;
static int mqtt_timeout;
