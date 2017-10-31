% Function to capture catvehicle velocity and plotting live graph
function velCapture(ROS_IP, roboname)
%If number of argument is not two, flag message and exit.
if nargin < 2
    disp('Uage: velocityProfiler(192.168.0.32, catvehicle)');
    return;
end
close all;
%rosshutdown;
modelname = strcat('/',roboname);
%Connect to ROS master
master_uri= strcat('http://',ROS_IP);
master_uri = strcat(master_uri,':11311');
%rosinit(master_uri);

%get handle for /catvehicle/vel topic for subscribing to the data
speedsub = rossubscriber(strcat(modelname,'/vel'));
dt = datestr(now,'mmmm-dd-yyyy-HH-MM-SS');
sprintf('Velocity capture starts at %s',dt)

t = 0:0.05:50;
output = zeros(length(t),1);
figure;
grid on;
title('Velocity [m/s]');
for i = 1:length(t)
    speedata = receive(speedsub,10);
    output(i) = speedata.Linear.X;
 
    plot([max(i-1,1),i], output([max(i-1,1),i]),'b-');
    hold on;
    drawnow;    
end
dt = datestr(now,'mmmm-dd-yyyy-HH-MM-SS');
file = strcat(dt,'.mat');
    save(file, 'output');
grid on;
title('Velocity [m/s]');
end
