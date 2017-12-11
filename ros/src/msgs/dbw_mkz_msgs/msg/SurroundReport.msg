Header header

# Cross Traffic Alert (CTA)
bool cta_left_alert
bool cta_right_alert
bool cta_left_enabled
bool cta_right_enabled

# Blind Spot Information System (BLIS)
bool blis_left_alert
bool blis_right_alert
bool blis_left_enabled
bool blis_right_enabled

# Sonar Sensors
bool sonar_enabled
bool sonar_fault

# Sonar ranges in meters, zero is no-detection
float32[12] sonar

# Sonar index enumeration
uint8 FRONT_LEFT_SIDE=0
uint8 FRONT_LEFT_CORNER=1
uint8 FRONT_LEFT_CENTER=2
uint8 FRONT_RIGHT_CENTER=3
uint8 FRONT_RIGHT_CORNER=4
uint8 FRONT_RIGHT_SIDE=5
uint8 REAR_LEFT_SIDE=6
uint8 REAR_LEFT_CORNER=7
uint8 REAR_LEFT_CENTER=8
uint8 REAR_RIGHT_CENTER=9
uint8 REAR_RIGHT_CORNER=10
uint8 REAR_RIGHT_SIDE=11
