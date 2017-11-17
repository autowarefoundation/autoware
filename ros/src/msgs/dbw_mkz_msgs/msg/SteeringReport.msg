Header header

# Steering Wheel
float32 steering_wheel_angle      # rad
float32 steering_wheel_angle_cmd  # rad
float32 steering_wheel_torque     # Nm

# Vehicle Speed
float32 speed                     # m/s

# Status
bool enabled  # Enabled
bool override # Driver override
bool timeout  # Command timeout

# Watchdog Counter
bool fault_wdc

# Faults
bool fault_bus1
bool fault_bus2
bool fault_calibration
