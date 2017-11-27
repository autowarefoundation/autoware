import React from 'react';
import ReactDOM from 'react-dom';

import ButtonRGL from "./button_rgl";
import ViewRGL from './view_rgl';
import Map2DView from './map_2d_view';
import RadarView from './radar_view';
import Map3DView from './map_3d_view';
import CameraView from './camera_view';


class Index extends React.Component {
    constructor() {
        super();
        this.state = {
            buttonRGL: {
                nodeWidth: 4,
                nodeHeight: 3,
                cols: 25,
                rowHeight: 9,
                nodes: [
                    {id: 1, x: 0,  y: 4, w: 4, h: 3, physics: false,  chosen: false, domain: "initialization", label: 'initialization', display: "Initialization", span: (<span>Initialization</span>)},
                    {id: 2, x: 5,  y: 0, w: 4, h: 3, physics: false, chosen: false, domain: "map", label: 'map', display: "Map", span: (<span>Map</span>)},
                    {id: 3, x: 10, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: "localization", label: 'localization', display: "Localization", span: (<span>Localization</span>)},
                    {id: 4, x: 15, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: "mission", label: 'mission', display: "Mission", span: (<span>Mission Planning</span>)},
                    {id: 5, x: 20, y: 0, w: 4, h: 3, physics: false, chosen: false, domain: "motion", label: 'motion', display: "Motion", span: (<span>Motion Planning</span>)},
                    {id: 6, x: 5,  y: 4, w: 4, h: 3, physics: false, chosen: false, domain: "sensing", label: 'sensing', display: "Sensing", span: (<span>Sensing</span>)},
                    {id: 7, x: 15, y: 4, w: 4, h: 3, physics: false, chosen: false, domain: "detection", label: 'detection', display: "Detection", span: (<span>Detection</span>)},
                    {id: 8, x: 0,  y: 0, w: 2, h: 3, physics: false,  chosen: false, domain: "rosbag", label: 'rosbag', display: "ROSBAG", span: (<span>ROSBAG</span>)},
                    {id: 9, x: 2,  y: 0, w: 2, h: 3, physics: false, chosen: false, domain: "rosbag", label: 'play', display: "Play", span: (<span>play</span>)},
                    {id: 10, x: 20, y: 4, w: 3, h: 3, physics: false, chosen: false, domain: "gateway", label: 'gateway', display: "Vehicle Gateway", span: (<span>Vehicle Gateway</span>)},
                    {id: 11, x: 23, y: 4, w: 1, h: 3, physics: false, chosen: false, domain: "gateway", label: 'on', display: "On", span: (<span>On</span>)},
                ],
                edges: [
                    {from: 1, to: 2, physics: true,  label: "Initialization -> Map"},
                    {from: 2, to: 3, physics: true,  label: "Map -> Localization"},
                    {from: 3, to: 4, physics: true,  label: "Localization -> Mission Planning"},
                    {from: 4, to: 5, physics: true,  label: "Mission Planning -> Motion Planning"},
                    {from: 1, to: 6, physics: true,  label: "Initialization -> Sensing"},
                    {from: 6, to: 3, physics: true,  label: "Sensing -> Localization"},
                    // {from: 6, to: 7, physics: true,  label: "Sensing -> Detection"},
                    {from: 7, to: 5, physics: false, label: "Detection -> Motion Planning"},
                    {from: 8, to: 9, physics: true,  label: "ROSBAG -> Play"},
                    {from: 5, to: 10, physics: true,  label: "Mission Planning -> Vehicle Gateway"},
                    {from: 10, to: 11, physics: true,  label: "Vehicle Gateway -> On"},
                ]
            },
            viewRGL: {
                cols: 25,
                rowHeight: 32,
                contents: [
                    {
                        layout: {
                            i: "map",
                            x: 0, y: 0, w: 12, h: 12,
                            isDraggable: false,
                            isResizable: false,
                        },
                        displayName: "Map",
                        buttonId: 2,
                        isVisible: false,
                        component: Map2DView,
                    },
                    {
                        layout: {
                            i: "radar",
                            x: 12, y: 0, w: 12, h: 12,
                            isDraggable: false,
                            isResizable: false,
                        },
                        displayName: "Radar",
                        buttonId: 6,
                        isVisible: false,
                        component: RadarView,
                    },
                    {
                        layout: {
                            i: "localization",
                            x: 12, y: 0, w: 12, h: 12,
                            isDraggable: false,
                            isResizable: false,
                        },
                        displayName: "Localization",
                        buttonId: 3,
                        isVisible: false,
                        component: Map3DView,
                    },
                    {
                        layout: {
                            i: "camera",
                            x: 0, y: 12, w: 12, h: 12,
                            isDraggable: false,
                            isResizable: false,
                        },
                        displayName: "Camera",
                        buttonId: 6,
                        isVisible: false,
                        component: CameraView,
                    },
                ]
            },
        };
    }
    render() {
        return (
            <div>
                <ButtonRGL
                    structure={this.state.buttonRGL}
                    updateStructure={this.updateButtonRGLStructure.bind(this)}
                />
                <ViewRGL
                    structure={this.state.viewRGL}
                    updateStructure={this.updateViewRGLStructure.bind(this)}
                />
            </div>
        );
    }
    updateButtonRGLStructure(nextStructure) {
        this.setState({buttonRGL: nextStructure});
        this.updateViewRGLVisibility();
    }
    updateViewRGLStructure(nextStructure) {
        this.setState({viewRGL: nextStructure});
    }
    updateViewRGLVisibility() {
        const viewRGL = this.state.viewRGL;
        const buttonRGL = this.state.buttonRGL;
        for(const contentIndex in viewRGL.contents) {
            const nodeId = viewRGL.contents[contentIndex].buttonId;
            const nodeIndex = buttonRGL.nodes.findIndex(node => node.id === nodeId);
            viewRGL.contents[contentIndex].isVisible = buttonRGL.nodes[nodeIndex].chosen;
        }
        this.setState({viewRGL: viewRGL});
    }
}

ReactDOM.render(
    <Index/>,
    document.getElementById('content')
);

