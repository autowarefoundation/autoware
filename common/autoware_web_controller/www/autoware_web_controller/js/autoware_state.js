if (!AutowareStateSubscriber) {
  var AutowareStateSubscriber = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.connect("ws://" + location.hostname + ":9090");

      var sub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/autoware/state",
        messageType: "autoware_auto_system_msgs/AutowareState",
      });

      sub.subscribe(function (message) {
        var res = "";
        if (message.state == 1) {
          res = "INITIALIZING";
        } else if (message.state == 2) {
          res = "WAITING_FOR_ROUTE";
        } else if (message.state == 3) {
          res = "PLANNING";
        } else if (message.state == 4) {
          res = "WAITING_FOR_ENGAGE";
        } else if (message.state == 5) {
          res = "DRIVING";
        } else if (message.state == 6) {
          res = "ARRIVED_GOAL";
        } else if (message.state == 7) {
          res = "FINALIZING";
        } else {
          res = "Undefined";
        }

        document.getElementById("autoware_state").innerHTML = res;
      });
    },
  };
  AutowareStateSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    AutowareStateSubscriber.ros.close();
  };
}
