% Author: Jonathan Sprinkle
% Copyright (c) 2016 Arizona Board of Regents
% See copyright file

% load the bagfile
bagfile=rosbag('catvehicle_tutorial.bag');
% select only the /catvehicle/odom topic
odomBag=select(bagfile,'Topic','/catvehicle/odom');
% extract timeseries data (this will take some time)
odom = timeseries(odomBag,'Pose.Pose.Position.X','Pose.Pose.Position.Y');

figure
% plot x,y vs. time
plot(odom.Time,odom.Data(:,1),odom.Time,odom.Data(:,2))
% legend information
legend({'X position','Y position'})

%% Select the commanded input value
% where are data interesting? Let's look at the cmd_vel_safe inputs
cmd_vel_safeBag=select(bagfile,'Topic','/catvehicle/cmd_vel_safe');
% this will take some time
cmd_vel_safe=timeseries(cmd_vel_safeBag,'Linear.X','Angular.Z');

%% Plot raw cmd_vel_safe data
% let's plot these data too
figure
% plot x,y vs. time
plot(cmd_vel_safe.Time,cmd_vel_safe.Data(:,1),cmd_vel_safe.Time,cmd_vel_safe.Data(:,2))
% legend information
legend({'Commanded velocity','Commanded steering'})

%% Find where interesting data start appearing 
veldiffs = diff(cmd_vel_safe.Data(:,1));
vindices = find(veldiffs);
cmd_vel_safe.Data(veldiffs(1)-5:veldiffs(1)+5,1)
deltadiffs = diff(cmd_vel_safe.Data(:,2));
dindices=find(deltadiffs);
cmd_vel_safe.Data(indices(1)-10:indices(1)+10,2)

disp('Note that the values within two time series will not necessarily');
disp(' match up wrt one another');
cmd_vel_safe.Time(vindices(1))
odom.Time(vindices(1))

%% find the index nearest to a certain time
interestingTime=cmd_vel_safe.Time(vindices(1));
odomTmp = odom.Time > interestingTime;
odomIndex = max(0,find(odomTmp)-5); % back up 5 indices, if they exist;


%% now, we plot side-by-side from our relative indices
figure
plot(odom.Time(odomIndex(1):end),odom.Data(odomIndex(1):end,1),...
    odom.Time(odomIndex(1):end),odom.Data(odomIndex(1):end,2),...
    cmd_vel_safe.Time(vindices(1):end),cmd_vel_safe.Data(vindices(1):end,1),...
    cmd_vel_safe.Time(vindices(1):end),cmd_vel_safe.Data(vindices(1):end,2));
legend({'X position','Y position','cmd velocity','cmd steering'})
    
%% visualize the (x,y) position only
figure
plot(odom.Data(:,1),odom.Data(:,2))
axis equal
legend('(x,y) position');
xlabel('X position')
ylabel('Y position')
