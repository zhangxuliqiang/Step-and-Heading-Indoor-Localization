clc
close all
clear variables

% folder_name = 'lopen1/';

% folder_name = 'okt 6 house session/lopen4/';

folder_name = 'around_the_block_samsung/';

folder_name = '3_times_around_the_block/';

% Location and date of data
location = struct('latitude', 52.0056634, 'longitude', 4.36677, 'altitude', 10);
date = struct('year', 2020, 'month', 05, 'day', 7);

%%% load Data %%%
shs_sample = loadAndroidDataset(['../datasets/' folder_name]);

% load calibration data
mag_calib_sample = loadAndroidDataset('../datasets/20200602_154324_calib_mag/');
% mag_calib_sample = loadAndroidDataset('../datasets/okt 6 house session/android mag calib/android_calib2/');
gyro_calib_sample = loadAndroidDataset('../datasets/calib_gyro_samsung/');
acc_calib_sample = loadAndroidDataset('../datasets/calib_ac_samsung/');

%% load magnetic north data

magnetic_north_sample = loadAndroidDataset('../datasets/magnetic_north/');

% magnetic_north_sample = loadAndroidDataset('../datasets/okt 6 house session/indoor magnetic north/');

% Create context from location, date and coordinateSystem
magnetic = findMagneticField(location, date);

%%
% Calibrating Sensor Data 
[~, accSampled, gyrSampled, magSampled, dev_comp_attitude,calib_mag_north] = ...
    calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);


%% Step detection
[shs.steps, shs.data, shs.sd_components] = stepDetection(accSampled, 'data' , false);

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

%% Estimating orientation

folder_name = 'finding_noise/';
noise_sample = loadAndroidDataset(['../datasets/' folder_name]);

[~, acc_noise, gyr_noise, mag_noise, ~,~] = ...
    calibrateSensors(noise_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);


variance.acc = max(var(acc_noise{:,:}));
variance.mag = max(var(mag_noise{:,:}));
variance.gyr = max(var(gyr_noise{:,:}));

%%

prior_est = [dev_comp_attitude{4,:}]';
% prior_est = [1,0,0,0]';

estimate1 = ExtendedKalmanFilter_series(prior_est, ...
                                accSampled, gyrSampled, magSampled, ...
                                calib_mag_north, variance, false);


clear target
target = timetable(estimate1.Time);
target.est = [estimate1.est{:,:}]';

[og_positions, step_orient] = plotTrajectory(target,shs);
%%
 target1 = timetable(shs_sample.device_computed.attitude.Time);
 target1.est = [shs_sample.device_computed.attitude{:,:}];

[dev_com_positions,dev_com_step_orient] = plotTrajectory(target1,shs);
%%
% [og_positions, step_orient] = plotTrajectory([estimate1.final_q{:,:}]',shs);



