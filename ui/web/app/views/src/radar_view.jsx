import React from 'react';
import ReactResizeDetector from 'react-resize-detector';
import { WEB_UI_URL } from "./dotenv";


export default class RadarView extends React.Component {
    constructor() {
        super();
        this.state = {};
    }
    render() {
        return (
            <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)} />
        );
    }
    componentDidMount() {
        //console.log("PointsRawView.componentDidMount", this.props, this.state);
        this.setState({points_raw: this.rosViewer(this.props.parentId, parseInt(this.props.width), parseInt(this.props.height))});
        // if(this.props.stop){
        //     this.state.points_raw.stop();
        // }
        // else{
        //     this.state.points_raw.start();
        // }
    }
    onDetectParentResize(w, h) {
        // console.log(["onDetectResize", w, h, this.state.points_raw]);
        if(typeof this.state.points_raw === 'undefined'){
            this.setState({points_raw: this.rosViewer(this.props.parentId, w, h)});
        }
        else{
            this.state.points_raw.resize(w, h);
        }
    }
    startView() {
        this.state.points_raw.start()
    }
    stopView() {
        this.state.points_raw.stop()
    }
    rosViewer(id, width, height) {
        // Connect to ROS.
        var ros = this.props.ros;

        // Create the main viewer.
        var viewer = new ROS3D.Viewer({
            divID : id,
            width : width,
            height : height,
            far: 2000,
            cameraPose: {x: -30, y: -20, z: 30},
            cameraZoomSpeed: 2.0,
            antialias : true
        });

        viewer.addObject(new ROS3D.Grid());

        // Setup a client to listen to TFs.
        var tfClient = new ROSLIB.TFClient({
            ros : ros,
            angularThres : 0.0001,
            transThres : 0.0001,
            rate : 5.0,
            fixedFrame : '/base_link'
        });

        var cloudClient = new ROS3D.PointCloud2({
            ros: ros,
            tfClient: tfClient,
            rootObject: viewer.scene,
            max_pts: 100000,
            topic: '/downsampled_points_raw'
        });

        // Setup the URDF client.
        var urdfClient = new ROS3D.UrdfClient({
            ros : ros,
            tfClient : tfClient,
            path : WEB_UI_URL+"/",
            rootObject : viewer.scene,
            loader : ROS3D.COLLADA_LOADER
        });

        return viewer;
    }
}
