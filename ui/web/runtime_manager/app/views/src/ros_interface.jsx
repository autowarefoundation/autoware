import { ROSBRIDGE_URL } from "./dotenv";

const getROSConnection = function() {
    var ros = new ROSLIB.Ros({url: ROSBRIDGE_URL});

    ros.on('connection', function () {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function (error) {
        console.error('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
    });

    return ros;
}

export { getROSConnection };
