import React from 'react';
import ReactDOM from 'react-dom';
import {getROSConnection} from "./ros_interface";
import Responsive, { WidthProvider } from 'react-grid-layout';
import ReactResizeDetector from 'react-resize-detector';
const ResponsiveReactGridLayout = WidthProvider(Responsive);


var hyphenOrRoundup = function(value) {
    return value=="-" ? value: Math.round(value*10.0)/10.0;
}

class Wrapper extends React.Component {
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

export default class ViewRGL extends React.Component {
    render() {
        const dndStyle = {
            position: 'absolute',
            left: '5px',
            top: 0,
            color: "white",
            cursor: 'pointer'
        };
        const layout = [];
        const viewComponents = [];
        //console.log(this.props.structure.contents);
        for(const content of this.props.structure.contents){
            //console.log(content);
            if(content.isVisible){
                layout.push(content.layout);
                viewComponents.push((
                    <Wrapper
                        key={content.layout.i}
                        id={content.layout.i}
                        style={{backgroundColor: "lightslategray"}}
                    >
                        <content.component
                            parentId={content.layout.i}
                            width={this.props.width}
                            height={this.props.height}
                            ros={getROSConnection()}
                            stop={false}
                        />
                    </Wrapper>
                ));
            }
        }
        //console.log("viewComponents", viewComponents);
        return (
            <div>
                <ResponsiveReactGridLayout
                    className="layout"
                    breakpoints={{lg: 1200, md: 996, sm: 768, xs: 480, xxs: 0}}
                    cols={this.props.structure.cols}
                    layout={layout}
                    rowHeight={this.props.structure.rowHeight}
                    ref="gridLayout"
                    onLayoutChange={this.onLayoutChange.bind(this)}
                >
                    {viewComponents}
                </ResponsiveReactGridLayout>
            </div>
        );
    }
    onLayoutChange(layout) {
    }
    onClick(e) {
    }
}
