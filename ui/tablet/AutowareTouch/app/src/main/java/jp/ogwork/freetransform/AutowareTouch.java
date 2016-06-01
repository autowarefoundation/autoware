package jp.ogwork.freetransform;

import android.content.Context;

import org.ros.message.*;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class AutowareTouch implements NodeMain {

    private ConnectedNode mNode;

    private int mVel;
    private int mGas;
    private int mBrake;
    private boolean mConn;
    private float mTwist;

    private MainActivity mMain;

    private Publisher<std_msgs.Float32> mAnglePublisher;
    private Publisher<std_msgs.Bool> mSetPublisher;
    private Publisher<std_msgs.Bool> mDrivePublisher;
    private Publisher<std_msgs.Bool> mNaviPublisher;
    private Publisher<std_msgs.Bool> mMapPublisher;
    private Publisher<std_msgs.Bool> mViewPublisher;
    private Publisher<std_msgs.Bool> mInfoPublisher;

    public AutowareTouch(Context context) {
        mMain = (MainActivity)context;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android_ros_sample/talker");
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    @Override
    public void onStart(ConnectedNode node) {
        mAnglePublisher = node.newPublisher("autoware_touch/angle", "std_msgs/Float32");
        mSetPublisher = node.newPublisher("autoware_touch/set", "std_msgs/Bool");
        mDrivePublisher = node.newPublisher("autoware_touch/drive", "std_msgs/Bool");
        mNaviPublisher = node.newPublisher("autoware_touch/navi", "std_msgs/Bool");
        mMapPublisher = node.newPublisher("autoware_touch/map", "std_msgs/Bool");
        mViewPublisher = node.newPublisher("autoware_touch/view", "std_msgs/Bool");
        mInfoPublisher = node.newPublisher("autoware_touch/info", "std_msgs/Bool");
        mNode = node;
        registerSubscriber();
    }

    @Override
    public void onShutdown(Node arg0) {
    }

    @Override
    public void onShutdownComplete(Node arg0) {
    }

    public void publishAngle(float f) {
        std_msgs.Float32 msg = mAnglePublisher.newMessage();
        msg.setData(f);
        mAnglePublisher.publish(msg);
    }

    public void publishSet(boolean b) {
        std_msgs.Bool msg = mSetPublisher.newMessage();
        msg.setData(b);
        mSetPublisher.publish(msg);
    }

    public void publishDrive(boolean b) {
        std_msgs.Bool msg = mDrivePublisher.newMessage();
        msg.setData(b);
        mDrivePublisher.publish(msg);
    }

    public void publishNavi(boolean b) {
        std_msgs.Bool msg = mNaviPublisher.newMessage();
        msg.setData(b);
        mNaviPublisher.publish(msg);
    }

    public void publishMap(boolean b) {
        std_msgs.Bool msg = mMapPublisher.newMessage();
        msg.setData(b);
        mMapPublisher.publish(msg);
    }

    public void publishView(boolean b) {
        std_msgs.Bool msg = mViewPublisher.newMessage();
        msg.setData(b);
        mViewPublisher.publish(msg);
    }

    public void publishInfo(boolean b) {
        std_msgs.Bool msg = mInfoPublisher.newMessage();
        msg.setData(b);
        mInfoPublisher.publish(msg);
    }

    private void registerSubscriber() {
        Subscriber<std_msgs.Int32> vel_sub = mNode.newSubscriber("autoware_touch/vel", "std_msgs/Int32");
        vel_sub.addMessageListener(
            new MessageListener<std_msgs.Int32>() {
                @Override
                public void onNewMessage(std_msgs.Int32 msg) {
                    mVel = msg.getData();
                    mMain.startDigitalHandle(Integer.toString(mVel));
                }
            }
        );

        Subscriber<std_msgs.Int32> gas_sub = mNode.newSubscriber("autoware_touch/gas", "std_msgs/Int32");
        gas_sub.addMessageListener(
            new MessageListener<std_msgs.Int32>() {
                @Override
                public void onNewMessage(std_msgs.Int32 msg) {
                    mGas = msg.getData();
                    mMain.startGasHandle(Integer.toString(mGas));
                }
            }
        );

        Subscriber<std_msgs.Int32> brake_sub = mNode.newSubscriber("autoware_touch/brake", "std_msgs/Int32");
        brake_sub.addMessageListener(
            new MessageListener<std_msgs.Int32>() {
                @Override
                public void onNewMessage(std_msgs.Int32 msg) {
                    mBrake = msg.getData();
                    mMain.startBrakeHandle(Integer.toString(mBrake));
                }
            }
        );

        Subscriber<std_msgs.Bool> conn_sub = mNode.newSubscriber("autoware_touch/conn", "std_msgs/Bool");
        conn_sub.addMessageListener(
            new MessageListener<std_msgs.Bool>() {
                @Override
                public void onNewMessage(std_msgs.Bool msg) {
                    mConn = msg.getData();
                    mMain.startConnHandle(Boolean.toString(mConn));
                }
            }
        );

        Subscriber<std_msgs.Float32> twist_sub = mNode.newSubscriber("autoware_touch/twist", "std_msgs/Float32");
        twist_sub.addMessageListener(
            new MessageListener<std_msgs.Float32>() {
                @Override
                public void onNewMessage(std_msgs.Float32 msg) {
                    mTwist = msg.getData();
                    mMain.startTwistHandle(Float.toString(mTwist));
                }
            }
        );
    }
}
