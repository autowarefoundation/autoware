# Steering Wheel
float32 steering_wheel_angle_cmd        # rad, range -8.2 to 8.2
float32 steering_wheel_angle_velocity   # rad/s, range 0 to 8.7, 0 = maximum

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Disable the driver override audible warning
bool quiet

# Watchdog counter (optional)
uint8 count
