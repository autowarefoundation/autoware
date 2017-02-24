% Author: Jonathan Sprinkle
% plots the distance outputs from a data file

function plotDistances
load distances.mat
% this timeseries is what we have
figure
hold on
plot(DistanceEstimator.Data__signal_1_);
plot(DistanceEstimator.Data__signal_2_);
legend({'Distance','Angle (rad)'});

end