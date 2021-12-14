if (!PathChangeApprovalPublisher) {
  var PathChangeApprovalPublisher = {
    ros: null,
    name: "",
    init: function () {
      this.ros = new ROSLIB.Ros();
      this.ros.on("error", function (error) {
        document.getElementById("path_change_approval_info").innerHTML = "Error";
      });
      this.ros.on("connection", function (error) {
        document.getElementById("path_change_approval_info").innerHTML = "Connected";
      });
      this.ros.on("close", function (error) {
        document.getElementById("path_change_approval_info").innerHTML = "Closed";
      });
      this.ros.connect("ws://" + location.hostname + ":9090");
    },
    send: function () {
      var pub = new ROSLIB.Topic({
        ros: this.ros,
        name: "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/path_change_approval",
        messageType: "tier4_planning_msgs/msg/Approval",
      });
      var str = new ROSLIB.Message({
        stamp: {
          sec: 0,
          nanosec: 0,
        },
        approval: true,
      });
      pub.publish(str);
    },
  };
  PathChangeApprovalPublisher.init();

  window.onload = function () {};
  window.onunload = function () {
    PathChangeApprovalPublisher.ros.close();
  };
}
if (!PathChangeApprovalStateSubscriber) {
  var PathChangeApprovalStateSubscriber = {
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
        name: "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/path_change_approval",
        messageType: "tier4_planning_msgs/msg/Approval",
      });
      sub.subscribe(function (message) {
        const div = document.getElementById("path_change_approval_status");
        if (div.hasChildNodes()) {
          div.removeChild(div.firstChild);
        }
        var res = message.command;
        var el = document.createElement("span");
        el.innerHTML = res;
        document.getElementById("path_change_approval_status").appendChild(el);
      });
    },
  };
  PathChangeApprovalStateSubscriber.init();

  window.onload = function () {};
  window.onunload = function () {
    PathChangeApprovalStateSubscriber.ros.close();
  };
}
