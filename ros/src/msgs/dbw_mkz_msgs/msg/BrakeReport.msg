Header header

# Brake pedal
# Unitless, range 0.15 to 0.50
float32 pedal_input
float32 pedal_cmd
float32 pedal_output

# Braking torque (Nm)
float32 torque_input
float32 torque_cmd
float32 torque_output

# Brake On Off (BOO), brake lights
bool boo_input
bool boo_cmd
bool boo_output

# Status
bool enabled  # Enabled
bool override # Driver override
bool driver   # Driver activity
bool timeout  # Command timeout

# Watchdog Counter
WatchdogCounter watchdog_counter
bool watchdog_braking
bool fault_wdc

# Faults
bool fault_ch1
bool fault_ch2
bool fault_boo
