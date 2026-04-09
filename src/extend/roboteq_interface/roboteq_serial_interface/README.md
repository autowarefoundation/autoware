# ROBOTEQ SERIAL INTERFACE

## Input / Output

### Input Topics

From roboteq_interface

| Name            | Type                                   | Description    |
| :-------------- | :------------------------------------- | :------------- |
| ~/input/command | roboteq_msgs/msg/RoboteqCommandStamped | input commands |

### Output Topics

To roboteq_interface

| Name            | Type                                  | Description   |
| :-------------- | :------------------------------------ | :------------ |
| ~/output/status | roboteq_msgs/msg/RoboteqStatusStamped | output status |

## ROS Parameter

| Name        | Type   | Description      | Default      |
| :---------- | :----- | :--------------- | :----------- |
| serial_port | string | serial port name | /dev/ttyUSB0 |

## Serial Communication

Transmitted data

- Motor Speed [rpm]
- Digital Output Control
  - DOut1 : connected to brake
  - DOut2 : connected to turn signal

Received data

- Motor Speed status [rpm]
- Fault Flag status
- Battery Voltage status
- Digital Input status
  - DIn1 : connected to emergency stop button
- Digital Output status
  - DOut1 : connected to brake
  - DOut2 : connected to turn signal
