# This is a message to hold data a GNSS unit
# Supported for MTi Devices with FW 1.4 and above.

std_msgs/Float64 itow
std_msgs/Float64 fix

float64 latitude
float64 longitude
float64 hEll
float64 hMsl

# ENU velocity
geometry_msgs/Vector3 vel

float64 hAcc
float64 vAcc
float64 sAcc

float64 pDop
float64 hDop
float64 vDop

float64 numSat
float64 heading
float64 headingAcc



