navigator.getUserMedia =
  navigator.getUserMedia ||
  navigator.webkitGetUserMedia ||
  navigator.mozGetUserMedia;
navigator.mediaDevices = navigator.mediaDevices || {};
navigator.mediaDevices.getUserMedia =
  navigator.mediaDevices.getUserMedia ||
  function (constraints) {
    return new Promise(function (resolve, reject) {
      navigator.getUserMedia(constraints, resolve, reject);
    });
  };
navigator.mediaDevices.enumerateDevices =
  navigator.mediaDevices.enumerateDevices ||
  function () {
    return new Promise(function (resolve) {
      MediaStreamTrack.getSources(function (devices) {
        for (var i = 0; i < devices.length; i++) {
          devices[i].deviceId = devices[i].id;
        }
        resolve(devices);
      });
    });
  };
navigator.mediaDevices.getSupportedConstraints =
  navigator.mediaDevices.getSupportedConstraints ||
  function () {
    return {
      aspectRatio: false,
      browserWindow: false,
      deviceId: true,
      facingMode: true,
      frameRate: false,
      groupId:false,
      height: true,
      mediaSource: false,
      sampleRate: false,
      sampleSize: false,
      scrollWithPage: false,
      volume: false,
      width: true
    }
  }

window.RTCSessionDescription = window.RTCSessionDescription || window.mozRTCSessionDescription;
window.RTCIceCandidate = window.RTCIceCandidate || window.mozRTCIceCandidate;
var orgRTCPeerConnection =
  window.RTCPeerConnection ||
  window.webkitRTCPeerConnection ||
  window.mozRTCPeerConnection;

if (orgRTCPeerConnection.prototype.createOffer.length) {
  window.RTCPeerConnection = function (configuration) {
    this.orgPC = new orgRTCPeerConnection(configuration);
    var that = this;

    // Async Methods
    this.createOffer = function (options) {
      return new Promise(function (resolve, reject) {
        that.orgPC.createOffer(resolve, reject, options);
      });
    };
    this.createAnswer = function () {
      return new Promise(function (resolve, reject) {
        that.orgPC.createAnswer(resolve, reject);
      });
    };
    this.setLocalDescription = function (description) {
      return new Promise(function (resolve, reject) {
        that.orgPC.setLocalDescription(description, resolve, reject);
      });
    };
    this.setRemoteDescription = function (description) {
      return new Promise(function (resolve, reject) {
        that.orgPC.setRemoteDescription(description, resolve, reject);
      });
    };
    this.addIceCandidate = function (iceCandidate) {
      return new Promise(function (resolve, reject) {
        that.orgPC.addIceCandidate(iceCandidate, resolve, reject);
      });
    };

    // Sync Methods
    this.updateIce = function (newConfiguration) {
      this.orgPC.updateIce(newConfiguration);
    };
    this.getConfiguration = function () {
      return this.orgPC.getConfiguration();
    };
    this.getLocalStreams = function () {
      return this.orgPC.getLocalStreams;
    };
    this.getRemoteStreams = function () {
      this.orgPC.getRemoteStreams;
    };
    this.getStreamById = function (streamId) {
      return this.orgPC.getStreamById(streamId);
    };
    this.addStream = function (stream) {
      this.orgPC.addStream(stream);
    };
    this.removeStream = function (stream) {
      this.orgPC.removeStream(stream);
    }
    this.close = function () {
      this.orgPC.close();
    };

    // Properties
    this.__defineGetter__('localDescription', function () {
      return this.orgPC.localDescription;
    });
    this.__defineGetter__('remoteDescription', function () {
      return this.orgPC.remoteDescription;
    });
    this.__defineGetter__('signalingState', function () {
      return this.orgPC.signalingState;
    });
    this.__defineGetter__('iceGatheringState', function () {
      return this.orgPC.iceGatheringState;
    });
    this.__defineGetter__('iceConnectionState', function () {
      return this.orgPC.iceConnectionState;
    });
    this.__defineGetter__('canTrickleIceCandidates', function () {
      return this.orgPC.canTrickleIceCandidates;
    });

    // Event Listeners
    this.__defineSetter__('onnegotiationneeded', function (func) {
      this.orgPC.onnegotiationneeded = func;
    });
    this.__defineSetter__('onicecandidate', function (func) {
      this.orgPC.onicecandidate = func;
    });
    this.__defineSetter__('onsignalingstatechange', function (func) {
      this.orgPC.onsignalingstatechange = func;
    });
    this.__defineSetter__('onaddstream', function (func) {
      this.orgPC.onaddstream = func;
    });
    this.__defineSetter__('onremovestream', function (func) {
      this.orgPC.onremovestream = func;
    });
    this.__defineSetter__('oniceconnectionstatechange', function (func) {
      this.orgPC.oniceconnectionstatechange = func;
    });
    this.__defineSetter__('onicegatheringstatechange', function (func) {
      this.orgPC.onicegatheringstatechange = func;
    });
  }
} else {
  window.RTCPeerConnection = orgRTCPeerConnection;
}

if (!('srcObject' in HTMLMediaElement.prototype)) {
  if (!('mozSrcObject' in HTMLMediaElement.prototype)) {
    HTMLMediaElement.prototype.__defineGetter__('srcObject', function (stream) {
      return this.streamSource;
    });
    HTMLMediaElement.prototype.__defineSetter__('srcObject', function (stream) {
      this.src = stream ? URL.createObjectURL(stream) : '';
      this.streamSource = stream;
    });
  } else {
    HTMLMediaElement.prototype.__defineGetter__('srcObject', function (stream) {
      return this.mozSrcObject;
    });
    HTMLMediaElement.prototype.__defineSetter__('srcObject', function (stream) {
      this.mozSrcObject = stream;
    });
  }
}
