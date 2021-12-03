if (!AutowareEngagePublisher) {
  var AutowareEngagePublisher = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("autoware_engage_info").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("autoware_engage_info").innerHTML = "Connected";
      });
      this.ros.on("close", function (error) {
        document.getElementById("autoware_engage_info").innerHTML = "Closed";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");
    },
    send: function (value) {
      var pub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/autoware/engage",
        messageType: "autoware_auto_vehicle_msgs/Engage",
        latch: "true",
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
  AutowareEngagePublisher.init();

  window.onload = function () {};
  window.onunload = function () {
    AutowareEngagePublisher.ros.close();
  };
}
if (!AutowareEngageStatusSubscriber) {
  var AutowareEngageStatusSubscriber = {
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
        name: "/autoware/engage",
        messageType: "autoware_auto_vehicle_msgs/Engage",
      });
      sub.subscribe(function (message) {
        const div = document.getElementById("autoware_engage_status");
        if (div.hasChildNodes()) {
          div.removeChild(div.firstChild);
        }
        var res = message.engage;
        var el = document.createElement("span");
        el.innerHTML = res;
        document.getElementById("autoware_engage_status").appendChild(el);
      });
    },
  };
  AutowareEngageStatusSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    AutowareEngageStatusSubscriber.ros.close();
  };
}
