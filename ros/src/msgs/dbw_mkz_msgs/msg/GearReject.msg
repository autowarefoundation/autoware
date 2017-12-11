uint8 value

uint8 NONE=0              # Not rejected
uint8 SHIFT_IN_PROGRESS=1 # Shift in progress
uint8 OVERRIDE=2          # Override on brake, throttle, or steering
uint8 ROTARY_LOW=3        # Rotary shifter can't shift to Low
uint8 ROTARY_PARK=4       # Rotary shifter can't shift out of Park
uint8 VEHICLE=5           # Rejected by vehicle (try pressing the brakes)
