/*
 * This file is auto-generated.  DO NOT MODIFY.
 * Original file: /home/usuda/src/workspace-adt-20141001/CanDataSender/src/com/metaprotocol/android/carlink_sample_receiver_service/ICarLinkDataReceiverService.aidl
 */
package com.metaprotocol.android.carlink_sample_receiver_service;
public interface ICarLinkDataReceiverService extends android.os.IInterface
{
/** Local-side IPC implementation stub class. */
public static abstract class Stub extends android.os.Binder implements com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService
{
private static final java.lang.String DESCRIPTOR = "com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService";
/** Construct the stub at attach it to the interface. */
public Stub()
{
this.attachInterface(this, DESCRIPTOR);
}
/**
 * Cast an IBinder object into an com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService interface,
 * generating a proxy if needed.
 */
public static com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService asInterface(android.os.IBinder obj)
{
if ((obj==null)) {
return null;
}
android.os.IInterface iin = obj.queryLocalInterface(DESCRIPTOR);
if (((iin!=null)&&(iin instanceof com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService))) {
return ((com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService)iin);
}
return new com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService.Stub.Proxy(obj);
}
@Override public android.os.IBinder asBinder()
{
return this;
}
@Override public boolean onTransact(int code, android.os.Parcel data, android.os.Parcel reply, int flags) throws android.os.RemoteException
{
switch (code)
{
case INTERFACE_TRANSACTION:
{
reply.writeString(DESCRIPTOR);
return true;
}
case TRANSACTION_setEventListener:
{
data.enforceInterface(DESCRIPTOR);
com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener _arg0;
_arg0 = com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener.Stub.asInterface(data.readStrongBinder());
int _result = this.setEventListener(_arg0);
reply.writeNoException();
reply.writeInt(_result);
return true;
}
case TRANSACTION_close:
{
data.enforceInterface(DESCRIPTOR);
int _result = this.close();
reply.writeNoException();
reply.writeInt(_result);
return true;
}
}
return super.onTransact(code, data, reply, flags);
}
private static class Proxy implements com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverService
{
private android.os.IBinder mRemote;
Proxy(android.os.IBinder remote)
{
mRemote = remote;
}
@Override public android.os.IBinder asBinder()
{
return mRemote;
}
public java.lang.String getInterfaceDescriptor()
{
return DESCRIPTOR;
}
@Override public int setEventListener(com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener listener) throws android.os.RemoteException
{
android.os.Parcel _data = android.os.Parcel.obtain();
android.os.Parcel _reply = android.os.Parcel.obtain();
int _result;
try {
_data.writeInterfaceToken(DESCRIPTOR);
_data.writeStrongBinder((((listener!=null))?(listener.asBinder()):(null)));
mRemote.transact(Stub.TRANSACTION_setEventListener, _data, _reply, 0);
_reply.readException();
_result = _reply.readInt();
}
finally {
_reply.recycle();
_data.recycle();
}
return _result;
}
@Override public int close() throws android.os.RemoteException
{
android.os.Parcel _data = android.os.Parcel.obtain();
android.os.Parcel _reply = android.os.Parcel.obtain();
int _result;
try {
_data.writeInterfaceToken(DESCRIPTOR);
mRemote.transact(Stub.TRANSACTION_close, _data, _reply, 0);
_reply.readException();
_result = _reply.readInt();
}
finally {
_reply.recycle();
_data.recycle();
}
return _result;
}
}
static final int TRANSACTION_setEventListener = (android.os.IBinder.FIRST_CALL_TRANSACTION + 0);
static final int TRANSACTION_close = (android.os.IBinder.FIRST_CALL_TRANSACTION + 1);
}
public int setEventListener(com.metaprotocol.android.carlink_sample_receiver_service.ICarLinkDataReceiverServiceCallbackListener listener) throws android.os.RemoteException;
public int close() throws android.os.RemoteException;
}
