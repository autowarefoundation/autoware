% Author: Jonathan Sprinkle
% Compares bag files from several experiments in gazebo using the same
% inputs

%% load the bagfiles

% realtime1000 = rosbag('simpleGazeboInputs/realtime1000.bag');
% realtime100  = rosbag('simpleGazeboInputs/realtime100.bag');
% slowtime1000 = rosbag('simpleGazeboInputs/slowtime1000.bag');
% slowtime100  = rosbag('simpleGazeboInputs/slowtime100.bag');

% realtime1000 = rosbag('complexGazeboInputs/realtime1000.bag');
% realtime100  = rosbag('complexGazeboInputs/realtime100.bag');
% slowtime1000 = rosbag('complexGazeboInputs/slowtime1000.bag');
% slowtime100  = rosbag('complexGazeboInputs/slowtime100.bag');
% realtime100_5000 = rosbag('complexGazeboInputs/realtime100-5000.bag');
% realtime100_5000_variant = rosbag('complexGazeboInputs/realtime100-5000-variant.bag');
% realtime100_5000_variant2 = rosbag('complexGazeboInputs/realtime100-5000-variant2.bag');

% realtime1000 = rosbag('complexGazeboInputs/realtime1000.bag');
% realtime100  = rosbag('complexGazeboInputs/realtime100.bag');
% slowtime1000 = rosbag('complexGazeboInputs/slowtime1000.bag');
% slowtime100  = rosbag('complexGazeboInputs/slowtime100.bag');

% badSteering = rosbag('complexGazeboInputs/hardleft-badSteering.bag');
% goodSteering= rosbag('complexGazeboInputs/hardleft-goodSteering.bag');
% goodSteeringSlomo= rosbag('complexGazeboInputs/hardleft-goodSteering-slomo.bag');
% goodSteeringSlomo2= rosbag('complexGazeboInputs/hardleft-goodSteering-slomo2.bag');
% 
% badSteering = rosbag('complexGazeboInputs/hardleft-badSteering.bag');
% goodSteering= rosbag('complexGazeboInputs/hardleft-goodSteering.bag');
% goodSteeringSlomo= rosbag('complexGazeboInputs/hardleft-goodSteering-slomo.bag');
% goodSteeringSlomo2= rosbag('complexGazeboInputs/hardleft-goodSteering-slomo2.bag');
% 

%% set up the cell arrays
bagfiles = cell(2,1);
ts = cell(2,1);

bagfiles{1} = rosbag('complexGazeboInputs/hardLeft-lessbounce1.bag');
bagfiles{2} = rosbag('complexGazeboInputs/hardLeft-lessbounce2.bag');
bagfiles{3} = rosbag('complexGazeboInputs/hardLeft-lessbounce3.bag');
% bagfiles{4} = rosbag('complexGazeboInputs/hardLeft-seed4.bag');
% bagfiles{5} = rosbag('complexGazeboInputs/hardLeft-seed5.bag');

% bagfiles{1} = realtime1000;
% bagfiles{2} = realtime100;
% bagfiles{3} = slowtime1000;
% bagfiles{4} = slowtime100;
% bagfiles{5} = realtime100_5000;
% bagfiles{6} = realtime100_5000_variant;
% bagfiles{7} = realtime100_5000_variant2;

%% make the cell of titles
titles = cell(2,1);
titles{1} = 'Run 1';
titles{2} = 'Run 2';
titles{3} = 'Run 3';
% titles{4} = 'Run 4';
% titles{5} = 'Run 5';
% titles{1} = 'Real time, 1000Hz';
% titles{2} = 'Real time, 100Hz';
% titles{3} = '1/10 time, 1000Hz';
% titles{4} = '1/10 time, 100Hz';
% titles{5} = 'Real time, 100Hz (5000 iters)';
% titles{6} = 'Real time, 100Hz (5000 iters, erp=0.1, cmcv=10, csl=0.01), #1';
% titles{7} = 'Real time, 100Hz (5000 iters, erp=0.1, cmcv=10, csl=0.01), #2';

%% grab data inputs of cmd_vel_safe
for i=1:length(bagfiles)
    bagselect = select(bagfiles{i}, 'Topic', '/catvehicle/odom');
    ts{i} = timeseries(bagselect, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');
end

%% produce a plot of data
figure
hold on

for i=1:length(bagfiles)
    plot(ts{i}.Data(:,2),ts{i}.Data(:,1));
end

legend(titles);
