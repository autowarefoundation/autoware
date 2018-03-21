var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var mqtt = require('mqtt');
var mqtt_client  = mqtt.connect('mqtt://localhost:1883');
var mqtt_subscribe_topics = ["/can_info", "/state", "/target_velocity", "/current_velocity", "/drive_mode"];

function convert_remote_cmd(remote_cmd) {
  var msg = remote_cmd["steering"] + "," +
  remote_cmd["accel"] + "," +
  remote_cmd["brake"] + "," +
  remote_cmd["gear"] + "," +
  remote_cmd["blinker"] + "," +
  remote_cmd["mode"] + "," +
  remote_cmd["control_mode"] + "," +
  remote_cmd["emergency"];

  return msg;
}

function set_subscribe_topics(mqtt_client, vehicle_id, mqtt_subscribe_topics) {
  for(topic of mqtt_subscribe_topics) {
    mqtt_client.subscribe('vehicle/' + vehicle_id + topic);
  }
}

app.use(express.static('public'));

app.get('/', function (req, res) {
  res.sendFile(__dirname + '/template/index.html');
});

app.get('/vehicle', function (req, res) {
  res.sendFile(__dirname + '/template/vehicle_window.html');
});

app.get('/demo_vehicle', function (req, res) {
  res.sendFile(__dirname + '/template/demo_vehicle_window.html');
});

app.get('/operator', function (req, res) {
  res.sendFile(__dirname + '/template/operator_window.html');
});

app.get('/demo_operator', function (req, res) {
  res.sendFile(__dirname + '/template/demo_operator_window.html');
});

io.on('connection', function (socket) {
  console.log('a user connected');
});

http.listen(9000, function () {
  console.log('listening on *:9000');
});

mqtt_client.on('connect', function () {
  // Develop
  // set_subscribe_topics(mqtt_client, "+", mqtt_subscribe_topics);
});

io.on('connection', function (socket) {
  socket.on('disconnect', function () {
    console.log(socket.roomName + " is disconnect!");
    socket.broadcast.to(socket.roomName).send('leave remotePeer');
    delete socket.roomName;
  });

  socket.on('join room', function (roomName) {
    if (roomName != null) {
      socket.join(roomName);
      socket.roomName = roomName;
      socket.send('{"roomName": "' + roomName + '"}');
      // Set MQTT Subscribe Topic
      set_subscribe_topics(mqtt_client, roomName, mqtt_subscribe_topics);
      console.log("roomName: "  + roomName);
      var roomMemberCount = io.sockets.adapter.rooms[roomName].length;
      if (roomMemberCount === 2) {
        process.nextTick(function () {
          io.sockets.in(roomName).send('ready');
        });
      } else if (roomMemberCount > 2) {
        socket.send('over');
      }
    } else {
      socket.send('roomName error');
      return;
    }
  });

  socket.on('message', function (message) {
    try{
      // ROOM
      if (socket.roomName) {
        socket.broadcast.to(socket.roomName).send(message);
      }
      // REMOTE CMD
      var remote_cmd = JSON.parse(message)["remote_cmd"];
      if(remote_cmd != null) {
        var msg = convert_remote_cmd(remote_cmd);
        mqtt_client.publish('vehicle/' + remote_cmd["vehicle_id"] + "/remote_cmd", msg);
      }
    }
    catch (e) {
    }
  });

  mqtt_client.on('message', function (topic, message) {
    try {
      var split_topic = topic.split("/");
      if (socket.roomName != null && socket.roomName == split_topic[1]) {
        var send_topic_name = "";
        for (var i = 2; i < split_topic.length; i++) {
          send_topic_name += "/";
          send_topic_name += split_topic[i];
        }
        if(mqtt_subscribe_topics.indexOf(send_topic_name) >= 0) {
          var msg = {};
          msg.vehicle_info = {};
          msg.vehicle_info.topic = send_topic_name;
          msg.vehicle_info.message = message.toString();
          socket.send(JSON.stringify(msg));
        }
      }
      else {
        // console.log("[NULL] " + topic + ": " + message.toString());
        // console.log("roomName: " + socket.roomName);
      }
    }
    catch (e) {
      console.error("mqtt_pubscriber", e.message);
    }
  })
});
