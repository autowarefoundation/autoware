%Implementation of follower algorithm

%Developed by Rahul Kumar Bhadani <rahulbhadani@email.arizona.edu>

%ROS_IP = IP Address of ROS Master
%lead = name of the model of leader AV Car
%follower =  name of the model of follower car

function follower_profile(ROS_IP, lead, follower)

%If number of argument is not two, flag message and exit.
if nargin < 2
    sprintf('Usage: velocityProfiler(192.168.0.32, catvehicle)');
    return;
end

rosshutdown;
close all;
modelname1 = strcat('/',lead);
modelname2 = strcat('/',follower);
%Connect to ROS master
master_uri= strcat('http://',ROS_IP);
master_uri = strcat(master_uri,':11311');
rosinit(master_uri);

%get handle for cmd_vel topic for publishing the data
velpub1 = rospublisher(strcat(modelname1,'/cmd_vel'),rostype.geometry_msgs_Twist);
velpub2 = rospublisher(strcat(modelname2,'/cmd_vel'),rostype.geometry_msgs_Twist);

%get handle for speed topic for subscribing to the data
speedsub1 = rossubscriber(strcat(modelname1,'/vel'));
speedsub2 = rossubscriber(strcat(modelname2,'/vel'));

%get handle for /DistanceEstimator
distanceEstimaterSub = rossubscriber('/DistanceEstimator/dist');

%Discretize timestamp
t = 0:0.05:150;
v1 = 3;
v2 = 6;
v3 = 0;

%Velocity profile
input = v1.*(t<50) + v2.*(t>=50).*(t<100) + v3.*(t>= 100);

%Velocity profile will be sine
%input = abs(2*sin(t));

%Variable to store output velocity
output1 = zeros(length(t),1);
output2 = zeros(length(t),1);

%handle for rosmessage object for velpub topic
velMsgs1 = rosmessage(velpub1);
velMsgs2 = rosmessage(velpub2);
for i=1:length(t)
    velMsgs1.Linear.X = input(i);
    velMsgs1.Angular.Z = 0.0;
    %Publish on the topic /catvehicle/cmd_vel
    send(velpub1, velMsgs1);
    %Read from the topic /catvehicle/speed
    speedata1 = receive(speedsub1,10);
    distance = receive(distanceEstimaterSub,10);
    x = distance.Data;
    
    %Follower control rule
    velMsgs2.Linear.X = (1/30.*x + 2/3).*speedata1.Linear.X;
    velMsgs2.Angular.Z = 0.0;
    send(velpub2, velMsgs2);
    speedata2 = receive(speedsub2,10);
    output1(i) = speedata1.Linear.X;
    output2(i) = speedata2.Linear.X;
end

%Plot the input and output velocity profile
[n, p] = size(output1);
T = 1:n;
plot(T, input');
hold on;
plot(T, output1);
plot(T, output2);
title('Original Data');
legend('Input function', 'Output response of lead','Output response of follower');
grid on;
