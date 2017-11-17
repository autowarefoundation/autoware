import React from 'react';
import ReactResizeDetector from 'react-resize-detector';
import { WEB_UI_URL } from "./dotenv";


export default class Map2DView extends React.Component {
    constructor() {
        super();
        this.state = {};
    }
    render() {
        // console.log("Map3DView.render()", this.props);
        return (
            <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)} />
        );
    }
    componentDidMount() {
        //console.log("Map3DView.componentDidMount", this.props, this.state);
        const points_map = this.rosViewer(this.props.parentId, parseInt(this.props.width), parseInt(this.props.height));
        this.setState({points_map: points_map});
        // if(this.props.stop){
        //     points_map.stop();
        // }
        // else{
        //     points_map.start();
        // }
    }
    onDetectParentResize(w, h) {
        // console.log("Map3DView.onDetectParentResize", w, h);
        if(typeof this.state.points_map === 'undefined'){
            this.setState({points_map: this.rosViewer(this.props.parentId, w, h)});
        }
        else{
            this.state.points_map.resize(w, h);
        }
    }
    rosViewer(id, width, height) {
        // Connect to ROS.
        var ros = this.props.ros;

        // Create the main viewer.
        var viewer = new ROS3D.Viewer({
            divID: id,
            width: width,
            height: height,
            background: "#333333",
            far: 4000,
            cameraPose: {x: -0.000001, y: 0, z: 500},
            cameraZoomSpeed: 0.0,
            antialias: true
        });

        viewer.addObject(new ROS3D.Grid());

        // Setup a client to listen to TFs.
        var tfClient = new ROSLIB.TFClient({
            ros: ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: '/world'
        });

        var tfClient = new ROSLIB.TFClient({
            ros: ros,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 5.0,
            fixedFrame: "/base_link"
        });

        var cloudClient = new ROS3D.PointCloud2({
            ros: ros,
            tfClient: tfClient,
            rootObject: viewer.scene,
            max_pts: 1000000,
            topic: '/downsampled_points_map'
        });

        var markerClientVectorMap = new ROS3D.MarkerArrayClient({
           ros: ros,
           tfClient: tfClient,
           rootObject: viewer.scene,
           topic: '/vector_map'
        });

        var urdfClient = new ROS3D.UrdfClient({
            ros: ros,
            tfClient: tfClient,
            path: WEB_UI_URL+"/",
            rootObject: viewer.scene,
            loader: ROS3D.COLLADA_LOADER
        });

        return viewer;
    }
}
