# Throttle pedal
# Options defined below
float32 pedal_cmd
uint8 pedal_cmd_type

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 count

uint8 CMD_NONE=0
uint8 CMD_PEDAL=1   # Unitless, range 0.15 to 0.80
uint8 CMD_PERCENT=2 # Percent of maximum throttle, range 0 to 1
