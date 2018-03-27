var MESSAGE_WAIT = 'Join the connection. Wait for new peer.';
var MESSAGE_REMOTE_LEAVE = 'Remote peer left. Wait for new peer.';
var MESSAGE_EXIST_STREAM = 'Your selected stream is already in use.';
var MESSAGE_OVER = 'Connection is empty. (one-on-one)';
var MESSAGE_ROOMNAME_ERROR = 'Connection name is not correct.';
var MESSAGE_NOTFOUND_DEVICE = 'Camera not found.)'
var MESSAGE_SIGNALING_SERVER_ERROR = 'Can not access signal server.';

// Development
var configuration = { iceServers: [{ url: 'stun:stun.l.google.com:19302'}, {urls: "stun:23.21.150.121" }] };
var signalingChannel = io.connect('http://' + location.host);
var pc;// = new RTCPeerConnection(configuration);
var roomName = '';
var actionName = '';
var localPositionNo = '0';
var remotePositionNo = '0';
var isFirefox = !!window.sidebar;
var mediaSenders = [];
var MAX_CAMERA_NUM = 6;
var audioStream = null;
var useDevices = {"front": null, "back": null};

othorSelectDevice.style.display = isFirefox ? 'none' : '';
firefoxSelectDevice.style.display = isFirefox ? '' : 'none';

var $messageDialog = $('#messageDialog');
var $selectDeviceDialog = $('#selectDeviceDialog');

// ---------------------------
// UI Process
// ---------------------------

// Set Event Listener -------

btnJoinRoom.addEventListener('click', setFocusTxtRoomName);
formJoinRoom.onsubmit = function (evt) {
  joinRoom();
  return false;
}
txtRoomName.onkeydown = function (evt) {
  (evt.keyCode === 13) && joinRoom();
}
//btnJoin.onclick = joinRoom;
for (var i = 1; i <= MAX_CAMERA_NUM; i++) {
  try {
    document.getElementById('btnAddStream' + i).addEventListener('click', showSelectDeviceDialog);
    document.getElementById('btnRemoveStream' + i).addEventListener('click', removeStream);
  }
  catch (e) {
  }
}
$messageDialog.on('shown.bs.modal', setFocusDialogRoomName);
$selectDeviceDialog.on('show.bs.modal', deviceChange);
btnFfSelectDevice.addEventListener('click', deviceChange);
btnDialogAddStream.addEventListener('click', addStream);
deviceList.addEventListener('change', deviceChange);


// Process Method -------------
function setFocusTxtRoomName() {
  setTimeout(function () {
    txtRoomName.focus();
  }, 100);
}

navigator.mediaDevices.enumerateDevices()
  .then(function (devices) {
    deviceList.innerHTML = '';
    var videoDevices = [];
    devices.forEach(function (device) {
      if (device.kind.indexOf('video') !== -1) {
        videoDevices.push(device);
      }
    });
    if (videoDevices.length) {
      createSelectItem(deviceList, '--Choose Your Device--', '');
      var cnt = 1;
      videoDevices.forEach(function (device) {
        createSelectItem(deviceList, device.label || 'camera-' + cnt, device.id || device.deviceId);
        cnt++;
      });
      deviceList.selectedIndex = 0;
    } else {
      console.log(MESSAGE_NOTFOUND_DEVICE);
      // showMessageDialog(MESSAGE_NOTFOUND_DEVICE);
      // btnJoinRoom.disabled = true;
    }
  });

function createSelectItem(selectElement, text, value) {
  var option = document.createElement("option");
  option.textContent = text;
  option.value = value;
  selectElement.appendChild(option);
}

function joinRoom() {
  $('#ddmJoinRoom').hide();
  if (txtRoomName.checkValidity()) {
    roomName = txtRoomName.value;
    if (roomName) {
      signalingChannel.emit('join room', roomName);
    }
  }
};

function showSelectDeviceDialog() {
  localPositionNo = this.dataset.no;
  deviceList.selectedIndex = 0;
  $selectDeviceDialog.modal('show');
}

function addStream() {
  actionName = 'add';
  var stream = streamPreview.srcObject;
  streamPreview.srcObject = null;
  $selectDeviceDialog.modal('hide');
  document.getElementById('btnAddStream' + localPositionNo).style.display = 'none';
  document.getElementById('btnRemoveStream' + localPositionNo).style.display = '';
  if (!pc) start();
  var localView = document.getElementById('localView' + localPositionNo);
  if (isFirefox) {
    stream.getTracks().forEach(function (track) {
      mediaSenders.push(pc.addTrack(track, stream));
    });
  } else {
    pc.addStream(stream);
  }
  localView.srcObject = stream;
  localView.play();

  // Set Camera ID
  console.log("ADD Strean");
  console.log(deviceList.value);
  deviceId = deviceList.value
  if(RUN_TYPE == "vehicle") {
    if(useDevices["front"] == null || useDevices["front"] == deviceId) {
      console.log("FrontCamera: " + deviceId)
      useDevices["front"] = deviceId;
    }
    else if((useDevices["front"] != null && useDevices["back"] == null) || useDevices["back"] == deviceId) {
      console.log("BackCamera: " + deviceId)
      useDevices["back"] = deviceId;
    }
    else {
      console.log("OtherCamera: " + deviceId)
    }
  }
}

function setFocusDialogRoomName() {
  dialogRoomName.focus();
  dialogRoomName.select();
}

function deviceChange() {
  var deviceId = deviceList.value;
  clearStream(streamPreview);

  if (deviceId || isFirefox) {
    // set audio device if fist device
    if(audioStream == null) {
      isUseAudio = true;
    }
    else {
      isUseAudio = false
    }
    deviceList.disabled = true;
    console.log("RUN_TYPE " + RUN_TYPE);
    if(RUN_TYPE == "vehicle") {
      if(useDevices["front"] == null || useDevices["front"] == deviceId) {
        console.log("FrontCamera: " + deviceId)
        var constraints = {
          audio: isUseAudio,
          video: {
            mandatory: {
              minAspectRatio: 1.777,
              maxAspectRatio: 1.778,
              minWidth: 320,
              minHeight: 180,
              minFrameRate: 30
            },
            optional: [{
              sourceId: deviceId
            }]
          }
        };
      }
      else if((useDevices["front"] != null && useDevices["back"] == null) || useDevices["back"] == deviceId) {
        console.log("BackCamera: " + deviceId)
        var constraints = {
          audio: isUseAudio,
          video: {
            mandatory: {
              minAspectRatio: 1.777,
              maxAspectRatio: 1.778,
              maxWidth: 768,
              maxHeight: 432,
              minFrameRate: 30
            },
            optional: [{
              sourceId: deviceId
            }]
          }
        };
      }
      else {
        console.log("OtherCamera: " + deviceId)
        var constraints = {
          audio: isUseAudio,
          video: {
            mandatory: {
              maxWidth: 640,
              maxHeight: 480,
              minWidth: 320,
              minHeight: 240,
              minFrameRate: 30
            },
            optional: [{
              sourceId: deviceId
            }]
          }
        };
      }
    }
    else if(RUN_TYPE == "operator") {
      var constraints = {
        audio: true,
        video: false
      };
    }
  }
  console.log(constraints);
  navigator.mediaDevices.getUserMedia(constraints)
    .then(function (stream) {
      if(isUseAudio == true) {
        audioStream = stream
      }
      streamPreview.srcObject = stream;
      streamPreview.play();
      deviceList.disabled = false;
    })
    .catch(function (err) {
      deviceList.disabled = false;
    });
}

function removeStream() {
  actionName = 'remove';
  localPositionNo = this.dataset.no;
  var stream = document.getElementById('localView' + localPositionNo).srcObject;
  // stream.stop();
  if(audioStream.id == stream.id) {
    audioStream = null;
  }
  if (isFirefox) {
    mediaSenders.forEach(function (sender) {
      pc.removeTrack(sender);
    });
    mediaSenders = [];
  } else {
    pc.removeStream(stream);
  }
}

function showMessageDialog(message, isWaiting, roomName) {
  $selectDeviceDialog.modal('hide');
  messageText.innerHTML = message;
  waitingIcon.setAttribute('aria-hidden', isWaiting ? 'false' : 'true');
  messageDialogFooter.style.display = isWaiting ? 'none' : '';
  roomName && message.replace('roomName', roomName);
  $messageDialog.modal('show');
}

function clearStream(video) {
  if (video.srcObject) {
    // video.srcObject.stop();
    video.srcObject = null;

  }
}

function clearStreamAll() {
  for (var i = 1; i <=5; i++) {
    document.getElementById('btnAddStream' + i).style.display = '';
    document.getElementById('btnRemoveStream' + i).style.display = 'none';
    clearStream(document.getElementById('localView' + i));
    clearStream(document.getElementById('remoteView' + i));
  }
}

// ---------------------------
// Signaling Process
// ---------------------------

signalingChannel.on('message', function (message) {
  if (message === 'ready') {
    $messageDialog.modal('hide');
    maskPanel.style.display = 'none';
  } else if (message === 'roomName error') {
    showMessageDialog(MESSAGE_ROOMNAME_ERROR);
  } else if (message === 'leave remotePeer') {
    clearStreamAll();
    pc = null;
    showMessageDialog(MESSAGE_REMOTE_LEAVE, true);
  } else if (message === 'over') {
    showMessageDialog(MESSAGE_OVER);
  } else {
    var message = JSON.parse(message);

    if (message.vehicle_info) {
      set_vehicle_info(message.vehicle_info);
    } else if (message.roomName) {
      // Join the room
      console.log("Vehicle ID: " + message.roomName);
      roomName = message.roomName;
      remote_cmd["vehicle_id"] = roomName;
      dialogRoomName.value = roomName;
      showMessageDialog(MESSAGE_WAIT, true);
      dialogRoomName.focus();
      dialogRoomName.select();
      ddJoinRoom.style.display = 'none';
    } else if (message.sdp) {
      if (!pc) start();
      remotePositionNo = message.streamPositionNo;
      var desc = new RTCSessionDescription(message.sdp);
      if (desc.type === 'offer') {
        pc.setRemoteDescription(desc).then(function () {
          return pc.createAnswer();
        })
        .then(function (answer) {
          return pc.setLocalDescription(answer);
        })
        .then(function () {
          signalingChannel.send(JSON.stringify({ sdp: pc.localDescription }));
        })
        .catch(logError);
      } else
        pc.setRemoteDescription(desc).catch(logError);
    } else if (message.candidate) {
      pc.addIceCandidate(new RTCIceCandidate(message.candidate)).catch(logError);
    }
  }
});


// ---------------------------
// WebRTC Process
// ---------------------------

function start() {
  if (isFirefox) {
    pc = new RTCPeerConnection();
  } else {
    pc = new RTCPeerConnection(configuration);
  }

  pc.onicecandidate = function (evt) {
    if (evt.candidate)
      signalingChannel.send(JSON.stringify({ candidate: evt.candidate }));
  };

  pc.onnegotiationneeded = function () {
    pc.createOffer()
      .then(function (offer) {
        return pc.setLocalDescription(offer);
      })
      .then(function () {
        signalingChannel.send(JSON.stringify({ sdp: pc.localDescription, streamPositionNo: localPositionNo }));
      })
      .then(function () {
        if (actionName === 'remove') {
          document.getElementById('btnAddStream' + localPositionNo).style.display = '';
          document.getElementById('btnRemoveStream' + localPositionNo).style.display = 'none';
          !isFirefox && clearStream(document.getElementById('localView' + localPositionNo));
        }
      })
      .catch(logError);
  };

  pc.onaddstream = function (evt) {
    var remoteView = document.getElementById('remoteView' + remotePositionNo);
    remoteView.srcObject = evt.stream;
    remoteView.play();
  };

  pc.onremovestream = function (evt) {
    var remoteView = document.getElementById('remoteView' + remotePositionNo);
    clearStream(remoteView);
  }
}

function start_AddEventListenerVer() {
  pc = new RTCPeerConnection(configuration);

  // send any ice candidates to the other peer
  pc.addEventListener('icecandidate', function (evt) {
    if (evt.candidate)
      signalingChannel.send(JSON.stringify({ candidate: evt.candidate }));
  });

  // let the "negotiationneeded" event trigger offer generation
  pc.addEventListener('negotiationneeded', function () {
    pc.createOffer()
      .then(function (offer) {
        return pc.setLocalDescription(offer);
      })
      .then(function () {
        // send the offer to the other peer
        signalingChannel.send(JSON.stringify({ sdp: pc.localDescription, streamPositionNo: localPositionNo }));
      })
      .then(function () {
        if (actionName === 'remove') {
          document.getElementById('btnAddStream' + localPositionNo).style.display = '';
          document.getElementById('btnRemoveStream' + localPositionNo).style.display = 'none';
          clearStream(document.getElementById('localView' + localPositionNo));
        }
      })
      .catch(logError);
  });

  // once remote stream arrives, show it in the remote video element
  pc.addEventListener('addstream', function (evt) {
    var remoteView = document.getElementById('remoteView' + remotePositionNo);
    remoteView.srcObject = evt.stream;
    remoteView.play();
  });

  pc.addEventListener('removestream', function (evt) {
    var remoteView = document.getElementById('remoteView' + remotePositionNo);
    clearStream(remoteView);
  });
}

function logError(error) {
  console.log(error);
}
