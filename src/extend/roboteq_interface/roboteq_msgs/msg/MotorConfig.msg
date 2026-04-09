builtin_interfaces/Time stamp
# max RPM
# min: 10, max: 65535, default: 1000
int16 max_rpm

# PID Gain
# P = kp/1,000,000  I = ki/1,000,000  D = kd/1,000,000
uint32 kp
uint32 ki
uint32 kd
