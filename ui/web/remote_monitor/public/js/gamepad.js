// gamepad
const DEFAULT_USE_DEVICE = "Logitech G29 Driving Force Racing Wheel (Vendor: 046d Product: c24f)";

const AXES_STEERING_INDEX = 0
const AXES_CLUTCH_PEDAL_INDEX = 1
const AXES_ACCEL_PEDAL_INDEX = 2
const AXES_BRAKE_PEDAL_INDEX = 3

let connectedGamepadIndex;
let loopID;

addOnload(
  function checkGamepads() {
    console.log("Init Gamepad");
    var gamepadList = navigator.getGamepads();

    for(var i=0; i<gamepadList.length; i++) {
      if(gamepadList[i] != null && gamepadList[i].id == DEFAULT_USE_DEVICE) {
        gamepad = gamepadList[i];
        connectedGamepadIndex = i;
        loopID = requestAnimationFrame(loop);
        console.log(gamepadList[i].id);
      }
    }
  }
);

function loop(timestamp) {
    let gamepads = navigator.getGamepads();
    let gp = gamepads[connectedGamepadIndex];

    remote_cmd["steering"] = - gp.axes[AXES_STEERING_INDEX];
    remote_cmd["accel"] = - (gp.axes[AXES_ACCEL_PEDAL_INDEX] - 1) / 2;
    remote_cmd["brake"] = - (gp.axes[AXES_BRAKE_PEDAL_INDEX] - 1) / 2;
    setSteeringPosition(-remote_cmd["steering"], 1, ".cfill")
    setAccelStroke(remote_cmd["accel"], 1, "controller_accel_bar");
    setBrakeStroke(remote_cmd["brake"], 1, "controller_brake_bar");

    requestAnimationFrame(loop);
}
