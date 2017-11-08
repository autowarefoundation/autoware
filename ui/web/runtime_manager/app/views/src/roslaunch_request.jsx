import React from 'react';
import { AxiosProvider, Request, Get, Delete, Head, Post, Put, Patch, withAxios } from 'react-axios';

export default class ROSLaunchRequest extends React.Component {
    render() {
        return (
            <div>
                <Get url={this.props.url}>
                    {(error, response, isLoading) => {
                        if(error) {
                            return this.props.errorCallback(error);
                        }
                        else if(isLoading) {
                            return this.props.isLoadingCallback(isLoading);
                        }
                        else if(response !== null) {
                            return this.props.responseCallback(response);
                        }
                        return this.props.defaultCallback();
                    }}
                </Get>
            </div>
        )
    }
}
