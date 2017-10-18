# Autoware Web UI

## Requirements

- MQTT
```
$ sudo add-apt-repository ppa:mosquitto-dev/mosquitto-ppa
$ sudo apt-get update
$ sudo apt-get install mosquitto mosquitto-clients
```

- nodejs
```
$ sudo apt-get install nodejs npm
$ npm install
```

- controler
  - Logicool G29 Driving Force

## Running
```
$ node app.js

# open http://localhost:9000/
# default vehicle id is 1.
# If you set vehicle id and mqtt setting, please see below code.
# Autoware/ros/src/socket/packages/mqtt_socket/include/mqtt_socket/mqtt_setting.hpp
```
