clc
close all
clear variables

%% % load Data %%%
 shs_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/lopen1_2/');

% load calibration data
calib_samples.mag_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/calib_mag/');
calib_samples.gyro_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');
calib_samples.acc_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/calib_acc/');

% load magnetic north data
magnetic_north_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');

% load noise sample
noise_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');

%% Calibrating Sensor Data 
[calib_data, dev_comp_attitude, variance] = ...
    calibrateSensors(shs_sample, calib_samples, ...
                     noise_sample, magnetic_north_sample);


%% Step detection
[shs.steps, shs.data, shs.sd_components] = stepDetection(calib_data.acc_data, 'data' , false);

% Step length estimation
shs.sl_components = timetable(shs.steps.data.Time);
shs.sl_components.period = [0; seconds(diff(shs.steps.data.Time))];
shs.sl_components.period(shs.sl_components.period == 0) = nan;
shs.sl_components.freq = 1./shs.sl_components.period;

male.k1 = 0.4;
male.k = 0.3116;

test_height = 1.78;

shs.steps.data.step_length = test_height.*male.k.*sqrt(shs.sl_components.freq);
shs.steps.data.step_length(1) = test_height.*male.k1;
shs.est_distance = sum(shs.steps.data.step_length);

%%
%  prior_est = [dev_comp_attitude{4,:}]';
prior_est = [1,0,0,0]';

ekf_estimate = ExtendedKalmanFilter_series(prior_est, calib_data, ...
    variance);

%%

door_handle_use = ReferenceFile2Timetable('datasets/marie testing/lopen1.2/lopen1_2/references.txt');

target = timetable(ekf_estimate.Time);
target.est = [ekf_estimate.est{:,:}]';
[og_positions1, step_orient1] = plotTrajectory(target,shs,door_handle_use);

clear target
target = timetable(shs_sample.device_computed.attitude.Time);
target.est = shs_sample.device_computed.attitude{:,:};
target = retime(target,unique(ekf_estimate.Time),'linear');
[og_positions1, step_orient1] = plotTrajectory(target,shs,door_handle_use);
