import React from 'react';

export default class Button extends React.Component {
    constructor() {
        super();
        this.state = {
            buttonStyle: {
                ini: {
                    outline: "none",
                    borderRadius: "10px",
                    wordWrap: "break-word",
                },
                on: {
                    backgroundColor: "#00a3e0",
                    color: "white",
                    border: "solid 2px #00a3e0",
                    textDecoration: "none",
                },
                off: {
                    backgroundColor: "white",
                    color: "#00a3e0",
                    border: "solid 2px #00a3e0",
                },
                disabled: {
                    backgroundColor: "DarkGray",
                    color: "Gray",
                    border: "solid 2px DarkGray",
                }
            }
        };
    }
    render() {
        return (
            <div>
                <button
                    type="button"
                    data-toggle="button"
                    aria-pressed="false"
                    style={this.getStyle.bind(this)()}
                    onClick={this.props.onClick}
                >
                    {this.props.span}
                </button>
            </div>
        );
    }
    getStyle() {
        const fontSize = Math.min(parseInt(this.props.height, 10)/1.5, parseInt(this.props.width, 10)/10) + "px";
        //console.log("fontSize", fontSize, this.props.height, this.props.width, Math.min(10*parseInt(this.props.height, 10), parseInt(this.props.width, 10)));
        const style = Object.assign(
            {},
            this.state.buttonStyle["ini"],
            this.state.buttonStyle[this.props.buttonState],
            {"width": this.props.width, "height": this.props.height, "fontSize": fontSize}
        );
        return style
    }
}
