%% step detection and counting - windowed peak detection 
%classical thompson peak detection approach

close all
clear all

data_directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Datasets/';
log_name = 'HIMU-2020-03-24_18-58-04.csv';

Acceleration = importHyperImuFile(strcat(data_directory, log_name));
Acceleration.Properties.VariableNames = ["X","Y","Z"];


%% plotting the raw data 
figure(1)
s1 = stackedplot(Acceleration);
title('3-axis raw accelerometer data for 8 steps')

%% create timetable for processed data
step_detection = timetable(Acceleration.timestamp);

%% generate the norm of the acceleration signal

step_detection.acc0_norm = sqrt(Acceleration.X.^2 + Acceleration.Y.^2 + ...
                    Acceleration.Z.^2);

% Detrending
acc_norm_mean = mean(step_detection.acc0_norm);

step_detection.acc1_detrend = step_detection.acc0_norm - acc_norm_mean;
%% thompson peak detection

% low pass filter 
lp_filter_num = [1,0,0,0,-1];
lp_filter_den = [1,-1];
lp_filter = (1/16)*filt(lp_filter_num, lp_filter_den)^2;

[step_detection.acc3_lp_filter,~] = lsim(lp_filter, step_detection.acc1_detrend);

% derivative operator
der_operator_num = [2,1,-1,-2];
der_operator_den = 1;

der_operator = filt(der_operator_num, der_operator_den);
[step_detection.acc4_deriv_filter, ~] = lsim(der_operator, step_detection.acc3_lp_filter);

% squaring
step_detection.acc5_square = step_detection.acc4_deriv_filter.^2;

% integration
N = 15;
integrator_num = (1/N) .* ones(1,N);
integrator_den = 1;

integrator  = filt(integrator_num, integrator_den);

[step_detection.acc6_integrator, ~] = lsim(integrator, step_detection.acc5_square);

%%
figure(2)
s2 = stackedplot(step_detection);
title("steps for thompson peak detection")

