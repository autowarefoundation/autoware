// Remote UI
const UPLOAD_INTERVAL = 25;
const MAX_STEERING_ANGLE = 600;
const MAX_ACCEL_STROKE = 1000;
const MAX_BRAKE_STROKE = 3000;
const EMERGENCY_OFF = 0;
const EMERGENCY_ON = 1;
const DRIVE_MODE_MANUAL = 0;
const DRIVE_MODE_STEER_PROGRAM = 1;
const DRIVE_MODE_DRIVE_PROGRAM = 2;
const DRIVE_MODE_ALL_PROGRAM = 3;
const MODE_AUTO_CONTROL = 1;
const MODE_REMOTE_CONTROL = 2;
const TOPIC_CAN_INFO = "/can_info";
const TOPIC_STATE = "/state";
const TOPIC_DRIVE_MODE = "/drive_mode";
const TOPIC_CURRENT_VELOCITY = "/current_velocity";
const TOPIC_TARGET_VELOCITY  = "/target_velocity";
const BUTTON_SWITCH_INTERVAL = 500;

let lastEmegencyUpdateTime = (new Date()).getTime();
let lastControlModeUpdateTime = (new Date()).getTime();

let remote_cmd = {
  "vehicle_id": 1,
  "steering": 0,
  "accel": 0,
  "brake": 0,
  "gear": 0,
  "blinker": 0,
  "emergency": EMERGENCY_OFF,
  "mode": 0,
  "control_mode": MODE_AUTO_CONTROL,
}

let vehicle_info = {}
let publish_flag = false;

// Add onload func
function addOnload(func)
{
    try {
        window.addEventListener("load", func, false);
    } catch (e) {
        // IE用
        window.attachEvent("onload", func);
    }
}

window.onload = function() {
  setInitialVal();
  setGamepadListener();

  var send_cmd = function(){
    if(remote_cmd["control_mode"] == MODE_REMOTE_CONTROL || remote_cmd["emergency"] == EMERGENCY_ON || publish_flag) {
      signalingChannel.send(JSON.stringify({ "remote_cmd": remote_cmd }));
      publish_flag = false;
    }
  }
  setInterval(send_cmd, UPLOAD_INTERVAL);
}

function setInitialVal() {
  setSteeringAngle(0);
  setSpeed(0);
  setRPM(0);
  setGear("P");
}

function set_vehicle_info(msg) {
  if(msg["topic"] == TOPIC_CAN_INFO) {
    vehicle_info = convert_can_info_csv_to_dict(msg["message"]);

    setSteeringAngle(-parseFloat(vehicle_info["angle"]), MAX_STEERING_ANGLE);
    setSteeringPosition(-parseFloat(vehicle_info["angle"]), MAX_STEERING_ANGLE);
    setSpeed(parseFloat(vehicle_info["speed"]));
    setRPM(parseFloat(vehicle_info["rpm"]));
    setGear(vehicle_info["driveshift"]);
    setAccelStroke(parseFloat(vehicle_info["drivepedal"]), MAX_ACCEL_STROKE);
    setBrakeStroke(parseFloat(vehicle_info["brakepedal"]), MAX_BRAKE_STROKE);
  }
  else if(msg["topic"] == TOPIC_DRIVE_MODE) {
    select_drive_mode_button(msg["message"]);
  }
  else if(msg["topic"] == TOPIC_STATE) {
    document.getElementById("text_state").innerHTML = "State: " + msg["message"];
  }
  else if(msg["topic"] == TOPIC_CURRENT_VELOCITY) {
    document.getElementById("text_target_speed").innerHTML = "Current Target Speed: " + msg["message"] + " km/h";
  }
  else if(msg["topic"] == TOPIC_TARGET_VELOCITY) {
    var velocity_sets = msg["message"].split(",");
    document.getElementById("text_next_target_speed").innerHTML = "Next Target Speed: " + velocity_sets[5] + " km/h";
  }
}

function select_gear(obj) {
  var idx = obj.selectedIndex;
  var value = obj.options[idx].value;
  var text  = obj.options[idx].text;

  console.log('value = ' + value + ', ' + 'text = ' + text);
  remote_cmd["gear"] = value;
}

function select_emergency_button(obj) {
  let currentUnixTime =  (new Date()).getTime();

  if (currentUnixTime - lastEmegencyUpdateTime > BUTTON_SWITCH_INTERVAL) {
    remote_cmd["emergency"] = remote_cmd["emergency"] == EMERGENCY_OFF ? EMERGENCY_ON : EMERGENCY_OFF;
    console.log('select_emergency_button => ' + remote_cmd["emergency"]);
    if(remote_cmd["emergency"] == EMERGENCY_ON) {
      document.getElementById("emergency_button").style.backgroundColor = "#FF0000";
      document.getElementById("emergency_button").innerHTML = "EMERGENCY ON";
    }
    else if(remote_cmd["emergency"] == EMERGENCY_OFF) {
      document.getElementById("emergency_button").style.backgroundColor = "#00a3e0";
      document.getElementById("emergency_button").innerHTML = "EMERGENCY";
    }
    publish_flag = true;
    lastEmegencyUpdateTime = currentUnixTime;
  }
}

function select_drive_mode_button(drive_mode) {
  if(drive_mode == DRIVE_MODE_MANUAL) {
    document.getElementById("drive_mode_button").style.backgroundColor = "#00a3e0";
    document.getElementById("drive_mode_button").innerHTML = "MANUAL";
  }
  else if(drive_mode == DRIVE_MODE_STEER_PROGRAM) {
    document.getElementById("drive_mode_button").style.backgroundColor = "#008000";
    document.getElementById("drive_mode_button").innerHTML = "STEER PROGRAM";
  }
  else if(drive_mode == DRIVE_MODE_DRIVE_PROGRAM) {
    document.getElementById("drive_mode_button").style.backgroundColor = "#008000";
    document.getElementById("drive_mode_button").innerHTML = "DRIVE PROGRAM";
  }
  else if(drive_mode == DRIVE_MODE_ALL_PROGRAM) {
    document.getElementById("drive_mode_button").style.backgroundColor = "#008000";
    document.getElementById("drive_mode_button").innerHTML = "PROGRAM";
  }
}


function select_mode_button(obj) {
  let currentUnixTime =  (new Date()).getTime();

  if (currentUnixTime - lastControlModeUpdateTime > BUTTON_SWITCH_INTERVAL) {
    remote_cmd["control_mode"] = remote_cmd["control_mode"] == MODE_AUTO_CONTROL ? MODE_REMOTE_CONTROL : MODE_AUTO_CONTROL;
    console.log('select_mode_button => ' + remote_cmd["control_mode"]);
    if(remote_cmd["control_mode"] == MODE_AUTO_CONTROL) {
      document.getElementById("control_mode_button").style.backgroundColor = "#00a3e0";
      document.getElementById("control_mode_button").innerHTML = "AUTO";
    }
    else if(remote_cmd["control_mode"] == MODE_REMOTE_CONTROL) {
      document.getElementById("control_mode_button").style.backgroundColor = "#008000";
      document.getElementById("control_mode_button").innerHTML = "REMOTE";
    }
    publish_flag = true;
    lastControlModeUpdateTime = currentUnixTime;
  }
}

// Rotate Image
function rotateImage(image_src, mime_type, angle) {
  var img = new Image();
  img.src = image_src;

  var min_size = 0;
  if (img.height <= img.width) {
    min_size = img.width;
  }
  else {
    min_size = img.height;
  }

  var newCanvas = document.createElement('canvas');
  newCanvas.width  = min_size;
  newCanvas.height = min_size;
  var newCtx = newCanvas.getContext('2d') ;
  newCtx.save();
  newCtx.translate(img.width / 2, img.height / 2) ;
  newCtx.rotate(angle);
  newCtx.drawImage ( img, -img.width / 2, -img.height / 2) ;
  // Image Base64
  return newCanvas.toDataURL(mime_type);
}

// Set Steering Angle
function setSteeringAngle(angle, max_angle) {
  var target_angle = 0;
  if (max_angle != null) {
    if (max_angle < angle) {
      angle = max_angle;
    }
    else if (angle < -max_angle) {
      angle = -max_angle;
    }
  }
  target_angle = angle * Math.PI / 180 % 360;

  document.getElementById('vehicle_steering').src = rotateImage("/img/steering.png", "image/png", target_angle);
}

// Set Speed
function setSpeed(speed) {
  var target_speed = 0;
  if (speed < 0) {
    target_speed = 0;
  }
  else if (220 < speed) {
    target_speed = 220;
  }
  else {
    target_speed = speed;
  }
  speedMeter.setValue(target_speed);
  document.getElementById("text_speed").innerHTML = "Speed: " + target_speed + " km/h";
}

// Set RPM
function setRPM(rpm) {
  var target_rpm = 0;
  if (rpm < 0) {
    target_rpm = 0;
  }
  else if (8000 < rpm) {
    target_rpm = 8000;
  }
  else {
    target_rpm = rpm;
  }
  rpmMeter.setValue(target_rpm);
}

// Set Gear
function setGear(gear) {
  gearMeter.innerHTML = gear;
}

// Set Accel Stroke
function setAccelStroke(value, max_value, bar_id) {
  if(bar_id == null) {
    bar_id = 'accel_bar'
  }
  var accel_bar = document.getElementById(bar_id);
  var target_value = 0;
  if (max_value != null) {
    target_value = 100 * value / max_value;
  }
  else {
    target_value = value;
  }
  accel_bar.value = target_value;
}

// Set Brake Stroke
function setBrakeStroke(value, max_value, bar_id) {
  if(bar_id == null) {
    bar_id = 'brake_bar'
  }
  var brake_bar = document.getElementById(bar_id);
  var target_value = 0;
  if (max_value != null) {
    target_value = 100 * value / max_value;
  }
  else {
    target_value = value;
  }
  brake_bar.value = target_value;
}

// Set Steering position
function setSteeringPosition(angle, max_angle, fill_name) {
  if(fill_name == null) {
    fill_name = ".fill";
  }
  var fill = document.querySelector(fill_name);
  var target_angle_ratio = 50;
  if (angle == 0) {
    target_angle_ratio = 50;
  }
  else if (0 < angle) {
    if (max_angle < angle) {
      target_angle_ratio = 100;
    }
    else {
      target_angle_ratio = 50 + 50 * (angle / max_angle);
    }
  }
  else if (angle < 0) {
    if (angle < -max_angle) {
      target_angle_ratio = 0;
    }
    else {
      target_angle_ratio = 50 + 50 * (angle / max_angle);
    }
  }
  fill.style.width = target_angle_ratio + '%';
}

// METER
var rpmMeter;
var speedMeter;
var gearMeter

var Meter = function Meter($elm, config) {

	// DOM
	var $needle = undefined,
	    $value = undefined;

	// Others

	var steps = (config.valueMax - config.valueMin) / config.valueStep,
	    angleStep = (config.angleMax - config.angleMin) / steps;

	var margin = 10; // in %
	var angle = 0; // in degrees

	var value2angle = function value2angle(value) {
		var angle = value / (config.valueMax - config.valueMin) * (config.angleMax - config.angleMin) + config.angleMin;

		return angle;
	};

	this.setValue = function (v) {
		$needle.style.transform = "translate3d(-50%, 0, 0) rotate(" + Math.round(value2angle(v)) + "deg)";
		$value.innerHTML = config.needleFormat(v);
	};

	var switchLabel = function switchLabel(e) {
		e.target.closest(".meter").classList.toggle('meter--big-label');
	};

	var makeElement = function makeElement(parent, className, innerHtml, style) {

		var e = document.createElement('div');
		e.className = className;

		if (innerHtml) {
			e.innerHTML = innerHtml;
		}

		if (style) {
			for (var prop in style) {
				e.style[prop] = style[prop];
			}
		}

		parent.appendChild(e);

		return e;
	};

	// Label unit
	makeElement($elm, "label label-unit", config.valueUnit);

	for (var n = 0; n < steps + 1; n++) {
		var value = config.valueMin + n * config.valueStep;
		angle = config.angleMin + n * angleStep;

		// Graduation numbers

		// Red zone
		var redzoneClass = "";
		if (value > config.valueRed) {
			redzoneClass = " redzone";
		}

		makeElement($elm, "grad grad--" + n + redzoneClass, config.labelFormat(value), {
			left: 50 - (50 - margin) * Math.sin(angle * (Math.PI / 180)) + "%",
			top: 50 + (50 - margin) * Math.cos(angle * (Math.PI / 180)) + "%"
		});

		// Tick
		makeElement($elm, "grad-tick grad-tick--" + n + redzoneClass, "", {
			left: 50 - 50 * Math.sin(angle * (Math.PI / 180)) + "%",
			top: 50 + 50 * Math.cos(angle * (Math.PI / 180)) + "%",
			transform: "translate3d(-50%, 0, 0) rotate(" + (angle + 180) + "deg)"
		});

		// Half ticks
		angle += angleStep / 2;

		if (angle < config.angleMax) {
			makeElement($elm, "grad-tick grad-tick--half grad-tick--" + n + redzoneClass, "", {
				left: 50 - 50 * Math.sin(angle * (Math.PI / 180)) + "%",
				top: 50 + 50 * Math.cos(angle * (Math.PI / 180)) + "%",
				transform: "translate3d(-50%, 0, 0) rotate(" + (angle + 180) + "deg)"
			});
		}

		// Quarter ticks
		angle += angleStep / 4;

		if (angle < config.angleMax) {
			makeElement($elm, "grad-tick grad-tick--quarter grad-tick--" + n + redzoneClass, "", {
				left: 50 - 50 * Math.sin(angle * (Math.PI / 180)) + "%",
				top: 50 + 50 * Math.cos(angle * (Math.PI / 180)) + "%",
				transform: "translate3d(-50%, 0, 0) rotate(" + (angle + 180) + "deg)"
			});
		}

		angle -= angleStep / 2;

		if (angle < config.angleMax) {
			makeElement($elm, "grad-tick grad-tick--quarter grad-tick--" + n + redzoneClass, "", {
				left: 50 - 50 * Math.sin(angle * (Math.PI / 180)) + "%",
				top: 50 + 50 * Math.cos(angle * (Math.PI / 180)) + "%",
				transform: "translate3d(-50%, 0, 0) rotate(" + (angle + 180) + "deg)"
			});
		}
	}

	// NEEDLE
	angle = value2angle(config.value);

	$needle = makeElement($elm, "needle", "", {
		transform: "translate3d(-50%, 0, 0) rotate(" + angle + "deg)"
	});

	var $axle = makeElement($elm, "needle-axle").addEventListener("click", switchLabel);
	makeElement($elm, "label label-value", "<div>" + config.labelFormat(config.value) + "</div>" + "<span>" + config.labelUnit + "</span>").addEventListener("click", switchLabel);

	$value = $elm.querySelector(".label-value div");
};

// DOM LOADED FIESTA

document.addEventListener("DOMContentLoaded", function () {

	rpmMeter = new Meter(document.querySelector(".meter--rpm"), {
		value: 6.3,
		valueMin: 0,
		valueMax: 8000,
		valueStep: 1000,
		valueUnit: "<div>RPM</div><span>x1000</span>",
		angleMin: 30,
		angleMax: 330,
		labelUnit: "RPM",
		labelFormat: function labelFormat(v) {
			return Math.round(v / 1000);
		},
		needleFormat: function needleFormat(v) {
			return Math.round(v / 100) * 100;
		},
		valueRed: 6500
	});

	speedMeter = new Meter(document.querySelector(".meter--speed"), {
		value: 0,
		valueMin: 0,
		valueMax: 100,
		valueStep: 10,
		valueUnit: "<span>Speed</span><div>Km/h</div>",
		angleMin: 30,
		angleMax: 330,
		labelUnit: "Km/h",
		labelFormat: function labelFormat(v) {
			return Math.round(v);
		},
		needleFormat: function needleFormat(v) {
			return Math.round(v);
		}
	});

	gearMeter = document.querySelector('.meter--gear div');
});
