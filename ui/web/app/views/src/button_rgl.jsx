import React from 'react';
import ReactDOM from 'react-dom';
import Responsive, { WidthProvider } from 'react-grid-layout';
const ResponsiveReactGridLayout = WidthProvider(Responsive);
import Button from "./button";
import ROSLaunchRequest from "./roslaunch_request";
import {WEB_UI_URL} from "./dotenv";

class GridSizeWrapper extends React.Component {
    render() {
        const that = this;
        const newChildren = React.Children.map(
            this.props.children,
            function(child) {
                return React.cloneElement(
                    child,
                    {
                        width: that.props.style.width,
                        height: that.props.style.height
                    }
                )
            }
        );
        return (
            <div
                {...this.props}
            >
                {newChildren}
            </div>
        );
    }
}


export default class ButtonRGL extends React.Component {
    getLayout() {
        const layout = [];
        for(const node of this.props.structure.nodes) {
            layout.push(
                {
                    i: node.id + "",
                    x: node.x,
                    y: node.y,
                    w: node.w,
                    h: node.h,
                    static: true}
            );
        }
        return layout;
    }
    getButtons() {
        const buttons = [];
        for(const node of this.props.structure.nodes) {
            buttons.push((
                <GridSizeWrapper key={node.id}>
                    <Button
                        span={node.span}
                        buttonState={node.physics ? (node.chosen ? "on" : "off") : "disabled"}
                        onClick={this.onClickButton.bind(this, node.id)}
                    />
                </GridSizeWrapper>
            ));
        }
        return buttons;
    }
    render() {
        return (
            <ResponsiveReactGridLayout
                className="layout"
                layout={this.getLayout()}
                cols={this.props.structure.cols}
                rowHeight={this.props.structure.rowHeight}
            >
                {this.getButtons()}
            </ResponsiveReactGridLayout>
        );
    }
    onClickButton(nodeId) {
        const index = this.props.structure.nodes.findIndex(node => node.id === nodeId);
        if(this.props.structure.nodes[index].physics) {
            const structure = this.props.structure;
            structure.nodes = this.getUpdatedNodes(
                nodeId,
                !this.props.structure.nodes[index].chosen,
                this.props.structure.nodes,
                this.props.structure.edges);

            const nodeLabel = structure.nodes[index].label;
            const nodeDisplay = structure.nodes[index].display;
            const url = WEB_UI_URL+"/roslaunch/"+nodeLabel+"/"+(this.props.structure.nodes[index].chosen ? "on" : "off");
            structure.nodes[index].span = (<ROSLaunchRequest
                url={url}
                errorCallback={() => { return (<span>error</span>); }}
                isLoadingCallback={() => { return (<span>{(this.props.structure.nodes[index].chosen ? "loading.." : "killing..")}</span>); }}
                responseCallback={() => { return (<span>{nodeDisplay}</span>); }}
                defaultCallback={() => { return (<span>{nodeDisplay}</span>); }}
            />);

            this.props.updateStructure(structure);
        }
    }
    getUpdatedNodes(nodeId, chosen, nodes, edges) {
        const index = nodes.findIndex(node => node.id === nodeId);
        nodes[index].chosen = chosen;
        if(chosen) {
            const toNodeIds = this.getToNodeIds(nodeId, edges);
            for(const toNodeId of toNodeIds) {
                const toNodeIndex = nodes.findIndex(node => node.id === toNodeId);
                nodes[toNodeIndex].physics = this.getToNodePhysics(toNodeId, nodes, edges);
            }
        }
        else {
            nodes = this.getDisabledNodes(nodeId, nodes, edges)
        }
        return nodes;
    }
    getToNodeIds(fromNodeId, edges){
        return edges.filter(
            (value) => { return value.from === fromNodeId; }
        ).map(
            (value, index, array) => { return value.to; }
        );
    }
    getToNodePhysics(toNodeId, nodes, edges){
        const fromNodeIds = edges.filter(
            (value) => { return value.physics && value.to === toNodeId; }
        ).map(
            (value, index, array) => { return value.from; }
        );
        if(fromNodeIds.length==0){
            return true;
        }
        if(fromNodeIds.length==1){
            return nodes[nodes.findIndex(node => node.id === fromNodeIds[0])].chosen;
        }
        return nodes.filter(
            (value) => { return fromNodeIds.indexOf(value.id) >= 0; }
        ).reduce(
            (previousValue, currentValue, index, array) => {
                return previousValue.chosen && currentValue.chosen;
            }
        );
    }
    getDisabledNodes(fromNodeId, nodes, edges) {
        const toNodeIds = this.getToNodeIds(fromNodeId, edges);
        for(const toNodeId of toNodeIds) {
            const toNodeIndex = nodes.findIndex(node => node.id === toNodeId);
            nodes[toNodeIndex].physics = this.getToNodePhysics(toNodeId, nodes, edges);
            if(!nodes[toNodeIndex].physics) {
                nodes[toNodeIndex].chosen = false;
                nodes = this.getDisabledNodes(nodes[toNodeIndex].id, nodes, edges);
            }
        }
        return nodes;
    }
}
