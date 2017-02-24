% Author: Jonathan Sprinkle
% plots the distance outputs from a data file

function plotData( timeseries )

% this timeseries is what we have
figure
hold on
plot(timeseries.dist);
plot(timeseries.velConverted);
plot(timeseries.vdot);
plot(timeseries.vout);
plot(timeseries.uTireAngle);
legend({'dist','velConverted','vdot','vout','uTireAngle'});

end