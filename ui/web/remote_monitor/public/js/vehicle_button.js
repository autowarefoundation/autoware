let blinker_status = [0, 0, 0];
let blinkers = ["blinker_left", "blinker_right", "blinker_hazard"];
let lastUpdateTime = (new Date()).getTime();

// blinker
function pushBlinker(blinker_id) {
  let currentUnixTime =  (new Date()).getTime();

  if (currentUnixTime - lastUpdateTime > BUTTON_SWITCH_INTERVAL) {
    blinker_status[blinker_id - 1] = (blinker_status[blinker_id - 1] == 1) ? 0 : 1;

    if(blinker_status[blinker_id - 1] == 1) {
      resetBlinkers();
      document.getElementById(blinkers[blinker_id - 1]).style.backgroundColor = "#FFFF00";
      remote_cmd["blinker"] = blinker_id;
    }
    else {
      document.getElementById(blinkers[blinker_id - 1]).style.backgroundColor = "#FFFFFF";
      remote_cmd["blinker"] = 0;
    }
    console.log("Blinker Status => "+ remote_cmd["blinker"]);
    lastUpdateTime = currentUnixTime;
  }
}

function resetBlinkers() {
  for(var i = 0; i < blinkers.length; i++) {
    document.getElementById(blinkers[i]).style.backgroundColor = "#FFFFFF";
  }
}
