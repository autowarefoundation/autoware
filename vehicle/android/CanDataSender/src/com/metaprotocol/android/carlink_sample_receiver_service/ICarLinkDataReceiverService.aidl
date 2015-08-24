package com.metaprotocol.android.carlink_sample_receiver_service;



import com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener;


interface ICarLinkDataReceiverService
{
	int setEventListener(ICarLinkDataReceiverServiceCallbackListener listener);

	int close();
}
