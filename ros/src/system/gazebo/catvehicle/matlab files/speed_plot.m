%   Author: Rahul Kumar Bhadani
%   rahulbhadani@email.arizona.edu
%   This file will read the velocity data from the ascii file and clip the
%   portion of data that are recorded before the input time so that we can
%   shift graph at t = t0 when experiment starts


    
% Load the data from file into matlab variable assuming that current
% directory has the data file named vData.mat. If you are working with file
% that has different name, please change the script

%Another assumption has been made that output start from zero and slowly
%gain the value.
data = load('vData.mat','-ascii');
%Step Input: Change the value as required
A =1;

[n, p] = size(data);
t = 1:n;
Input=repmat(A,1,n);
figure(1);
plot(t, Input');
hold on;
plot(t, data);
title('Original Data');
legend('Input Step function', 'Output response');
grid on;

%determine the size of data file
first_zero = 0;
last_zero = 0;
final_time = 0;

%Clip the region before the start of the experiment 
for t=1:length(data)
    if( data(t) > 0.0 && data (t) < 0.0001)
        first_zero = t;
    end
end

data =  data(first_zero:length(data));

for t=1:length(data)
    if( data(t+5) - data (t) > 0.0005)
        last_zero = t;
        break;
    end
end

data =  data(last_zero:length(data));

for t=1:length(data)
    if( abs(data(t+10) - data (t)) < 0.000001)
        final_time = t+10;
        break;
    end
end

data =  data(1:final_time);

%At this point we have the data that are from experiment.
[n, p] = size(data);
t = 1:n;
Input=repmat(A,1,n);
figure(2);
plot(t, Input');
hold on;
plot(t, data);
title('Clean Data');
legend('Input Step function', 'Output response');
grid on;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Approximation of data as second order system to determine the transfer
%function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Get the peak time.
[maxVal, maxTime] = max(data);
peakTime = maxTime;
hold on;
plot(peakTime, maxVal,'*');
text(peakTime, maxVal,'Y_{peak}');
finalVal = 0;
finalTime = 0;
%Find the final value. If the amplitude has not changed more than threshold
%within last 50
%unit of time, treat the latter amplitude as final value.
 for t=peakTime:length(data)
     if( abs(data(t) - data (t-25)) < A*0.0001)
         finalVal = data(t);
         finalTime = t;
         break;
     end
 end
 
plot(finalTime, finalVal,'*');
text(finalTime, finalVal,'Y_{final}');

%Overshoot
Overshoot = (maxVal - finalVal)/finalVal;
%Damping Coefficient
Zeta = sqrt( (log(Overshoot)^2) / ( (log(Overshoot)^2) +(pi^2) ) );

Wd = pi/peakTime;
Wn = Wd/(  sqrt ( 1 - (Zeta^2) )  );

denom = [1 2*Zeta*Wn Wn*Wn];
sys = tf(A, denom);
t = [0:0.01:1000];
figure(3);
title('Approximation of response as second order system');
step(sys,t);
grid on;
