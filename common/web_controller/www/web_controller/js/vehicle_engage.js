if (!VehicleEngagePublisher) {
  var VehicleEngagePublisher = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("state").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("state").innerHTML = "Connected";
      });
      this.ros.on("close", function (error) {
        document.getElementById("state").innerHTML = "Closed";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");
    },
    send: function (value) {
      var pub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/vehicle/engage",
        messageType: "autoware_auto_vehicle_msgs/Engage",
      });

      if (value == "Engage") {
        var str = new ROSLIB.Message({
          stamp: {
            sec: 0,
            nanosec: 0,
          },
          engage: true,
        });
        pub.publish(str);
      } else {
        var str = new ROSLIB.Message({
          stamp: {
            sec: 0,
            nanosec: 0,
          },
          engage: false,
        });
        pub.publish(str);
      }
    },
  };
  VehicleEngagePublisher.init();

  window.onload = function () {};
  window.onunload = function () {
    VehicleEngagePublisher.ros.close();
  };
}
if (!VehicleDisengagePublisher) {
  var VehicleDisengagePublisher = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("state").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("state").innerHTML = "Connected";
      });
      this.ros.on("close", function (error) {
        document.getElementById("state").innerHTML = "Closed";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");
    },
    send: function () {
      var pub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/vehicle/engage",
        messageType: "autoware_auto_vehicle_msgs/Engage",
      });

      var str = new ROSLIB.Message({
        stamp: {
          sec: 0,
          nanosec: 0,
        },
        engage: false,
      });
      pub.publish(str);
    },
  };
  VehicleDisengagePublisher.init();

  window.onload = function () {};
  window.onunload = function () {
    VehicleDisengagePublisher.ros.close();
  };
}
if (!VehicleEngageStatusSubscriber) {
  var VehicleEngageStatusSubscriber = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("state").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("state").innerHTML = "Connect";
      });
      this.ros.on("close", function (error) {
        document.getElementById("state").innerHTML = "Close";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");

      var sub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/vehicle/engage",
        messageType: "autoware_auto_vehicle_msgs/Engage",
      });
      sub.subscribe(function (message) {
        const div = document.getElementById("vehicle_engage_status");
        if (div.hasChildNodes()) {
          div.removeChild(div.firstChild);
        }
        var res = message.engage;
        var el = document.createElement("span");
        el.innerHTML = res;
        document.getElementById("vehicle_engage_status").appendChild(el);
      });
    },
  };
  VehicleEngageStatusSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    VehicleEngageStatusSubscriber.ros.close();
  };
}
if (!VehicleControlModeStatusSubscriber) {
  var VehicleControlModeStatusSubscriber = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("state").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("state").innerHTML = "Connect";
      });
      this.ros.on("close", function (error) {
        document.getElementById("state").innerHTML = "Close";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");

      var sub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/vehicle/status/control_mode",
        messageType: "autoware_auto_vehicle_msgs/ControlModeReport",
      });
      sub.subscribe(function (message) {
        const div = document.getElementById("vehicle_control_mode_status");
        if (div.hasChildNodes()) {
          div.removeChild(div.firstChild);
        }
        var res = "";
        if (message.mode == 0) {
          res = "No command";
        } else if (message.mode == 1) {
          res = "Autonomous";
        } else if (message.mode == 2) {
          res = "Manual";
        } else if (message.mode == 3) {
          res = "Disengaged";
        } else if (message.mode == 4) {
          res = "Not ready";
        } else {
          res = "Undefined";
        }
        var el = document.createElement("span");
        el.innerHTML = res;
        document.getElementById("vehicle_control_mode_status").appendChild(el);
      });
    },
  };
  VehicleControlModeStatusSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    VehicleControlModeStatusSubscriber.ros.close();
  };
}
