
window.RTCSessionDescription = window.RTCSessionDescription || window.mozRTCSessionDescription;
window.RTCIceCandidate = window.RTCIceCandidate || window.mozRTCIceCandidate;
window.RTCPeerConnection = window.mozRTCPeerConnection;
HTMLMediaElement.prototype.__defineGetter__('srcObject', function (stream) {
  return this.mozSrcObject;
});
HTMLMediaElement.prototype.__defineSetter__('srcObject', function (stream) {
  this.mozSrcObject = stream;
});
