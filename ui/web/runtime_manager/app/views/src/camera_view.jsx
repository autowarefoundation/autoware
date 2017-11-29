import React from 'react';
import ReactResizeDetector from 'react-resize-detector';
import { ROSVIDEO_URL } from "./dotenv";

export default class CameraView extends React.Component {
    constructor(props) {
        super(props);
        this.state = {
            width: parseInt(this.props.width, 10),
            height: parseInt(this.props.height, 10)
        };
    }
    render() {
        const url = ROSVIDEO_URL+"/stream?topic=/image_raw&width="+this.state.width+"&height="+this.state.height+"&type=mjpeg&quality=20";
        return (
            <div>
                <ReactResizeDetector handleWidth handleHeight onResize={(w, h)=>this.onDetectParentResize(w, h)}/>
                <img src={url}/>
            </div>
        );
    }
    onDetectParentResize(w, h) {
        if(6 < Math.abs(h-this.state.height)) {
            this.setState({height: h});
        }
        this.setState({width: w}); 
    }
}

