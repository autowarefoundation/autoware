// gamepad
const DEFAULT_USE_DEVICE = "Logitech G29 Driving Force Racing Wheel (Vendor: 046d Product: c24f)";

const AXES_STEERING_INDEX = 0
const AXES_CLUTCH_PEDAL_INDEX = 1
const AXES_ACCEL_PEDAL_INDEX = 2
const AXES_BRAKE_PEDAL_INDEX = 3

let connectedGamepadIndex;
let loopID;
let isInitGamepad = false;

function setGamepadListener() {
  console.log("Init GamepadListener");
  loopID = requestAnimationFrame(loop);
}

function loop(timestamp) {
  let gamepadList = navigator.getGamepads();

  // if non initialized
  if(isInitGamepad == false) {
    for(var i=0; i<gamepadList.length; i++) {
      if(gamepadList[i] != null && gamepadList[i].id == DEFAULT_USE_DEVICE) {
        gamepad = gamepadList[i];
        console.log("Init Gamepad");
        connectedGamepadIndex = i;
        isInitGamepad = true;
      }
    }
  }

  // get gamepad value
  if(isInitGamepad == true) {
    let gp = gamepadList[connectedGamepadIndex];

    // Steering, Accel, Brake
    remote_cmd["steering"] = - gp.axes[AXES_STEERING_INDEX];
    remote_cmd["accel"] = - (gp.axes[AXES_ACCEL_PEDAL_INDEX] - 1) / 2;
    remote_cmd["brake"] = - (gp.axes[AXES_BRAKE_PEDAL_INDEX] - 1) / 2;
    setSteeringPosition(-remote_cmd["steering"], 1, ".cfill")
    setAccelStroke(remote_cmd["accel"], 1, "controller_accel_bar");
    setBrakeStroke(remote_cmd["brake"], 1, "controller_brake_bar");

    // right lamp
    if(gp.buttons[4].value == 1) {
      pushBlinker(2);
    }
    // left lamp
    else if(gp.buttons[5].value == 1) {
      pushBlinker(1);
    }
    // hazard lamp
    else if(gp.buttons[3].value == 1) {
      pushBlinker(3);
    }

    // EMERGENCY
    if(gp.buttons[23].value == 1) {
      select_emergency_button();
    }

    // Control MODE
    if(gp.buttons[24].value == 1) {
      select_mode_button();
    }
  }
  requestAnimationFrame(loop);
}
