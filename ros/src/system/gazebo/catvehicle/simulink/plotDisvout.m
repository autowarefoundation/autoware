% Author: Jonathan Sprinkle
% plots the distance outputs from a data file

function plotData( timeseries )

% this timeseries is what we have
figure
hold on
plot(timeseries.Data);
plot(timeseries.uVelOut);
legend({'Distance','VelOut'});

end