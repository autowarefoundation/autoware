#include "std_msgs/String.h"

#define VEHICLEID       "1"
#define ADDRESS         "tcp://localhost:1883"
#define CLIENTID        "vehicle"
#define SENDER_TOPIC    "vehicle/"
#define RECIEVER_TOPIC  "vehicle/"
#define PAYLOAD         "Autoware"
#define QOS             1
#define TIMEOUT         10000L
#define AUTO_MODE       1
#define REMOTE_MODE     2
#define NORMAL_MODE     0
#define EMERGENCY_MODE  1
#define ACCEL_MAX_VAL   5 // km/h
#define BRAKE_MAX_VAL   10000
#define STEER_MAX_VAL   0.6
#define DOWNSAMPLE      0.1
