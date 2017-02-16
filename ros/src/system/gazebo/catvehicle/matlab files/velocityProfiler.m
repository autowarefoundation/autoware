%Matlab scripto to publish velocity on /catvehicle/cmd_vel topic and
%subscribe to catvehicle/speed topic

%Developed by Rahul Kumar Bhadani <rahulbhadani@email.arizona.edu>

%ROS_IP = IP Address of ROS Master
%roboname = name of the model
function velocityProfiler(ROS_IP, roboname, tire_angle)


%If number of argument is not two, flag message and exit.
if nargin < 2
    sprintf('Uage: velocityProfiler(192.168.0.32, catvehicle)');
    return;
end

if nargin < 3
    tire_angle = 0.0;
end
rosshutdown;
close all;
modelname = strcat('/',roboname);
%Connect to ROS master
master_uri= strcat('http://',ROS_IP);
master_uri = strcat(master_uri,':11311');
rosinit(master_uri);

%get handle for /catvehicle/cmd_vel topic for publishing the data
velpub = rospublisher(strcat(modelname,'/cmd_vel'),rostype.geometry_msgs_Twist);

%get handle for /catvehicle/vel topic for subscribing to the data
speedsub = rossubscriber(strcat(modelname,'/vel'));

%Discretize timestamp
t = 0:0.01:150;
v1 = 3;
v2 = 6;
v3 = 0;

%Velocity profile
input = v1.*(t<50) + v2.*(t>=50).*(t<100) + v3.*(t>= 100);

%Velocity profile will be sine
%input = abs(2*sin(t));

%Variable to store output velocity
output = zeros(length(t),1);

%handle for rosmessage object for velpub topic
velMsgs = rosmessage(velpub);
for i=1:length(t)
    velMsgs.Linear.X = input(i);
    velMsgs.Angular.Z = tire_angle;
    %Publish on the topic /catvehicle/cmd_vel
    send(velpub, velMsgs);
    %Read from the topic /catvehicle/speed
    speedata = receive(speedsub,10);
    output(i) = speedata.Linear.X;
end

%Plot the input and output velocity profile
[n, p] = size(output);
T = 1:n;
plot(T, input');
hold on;
plot(T, output);
title('Original Data');
legend('Input function', 'Output response');
grid on;
