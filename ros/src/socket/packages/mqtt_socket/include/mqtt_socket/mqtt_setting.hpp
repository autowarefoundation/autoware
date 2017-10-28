#include "std_msgs/String.h"

#define VEHICLEID       "1"
#define MQTT_ADDRESS    "localhost"
#define MQTT_PORT       1883
#define CLIENTID        "vehicle"
#define SENDER_TOPIC    "vehicle/"
#define RECIEVER_TOPIC  "vehicle/"
#define PAYLOAD         "Autoware"
#define QOS             0
#define TIMEOUT         10000L
#define AUTO_MODE       1
#define REMOTE_MODE     2
#define NORMAL_MODE     0
#define EMERGENCY_MODE  1
#define ACCEL_MAX_VAL   100
#define BRAKE_MAX_VAL   10000
#define STEER_MAX_VAL   0.6
#define LINEAR_X_MAX_VAL 15 * 1000 / 60 / 60 // 15 km/h
#define DOWNSAMPLE      0.1
#define GEAR_D          16
#define GEAR_N          32
#define GEAR_R          64
#define GEAR_P          128
