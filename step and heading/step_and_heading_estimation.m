clc
close all
clear variables

folder_name = '3_times_around_the_block/';

% folder_name = 'around_the_block_samsung/';

% Location and date of data
location = struct('latitude', 52.0056634, 'longitude', 4.36677, 'altitude', 10);
date = struct('year', 2020, 'month', 05, 'day', 7);

%%% load Data %%%
shs_sample = loadAndroidDataset(['../datasets/' folder_name]);

% load calibration data
mag_calib_sample = loadAndroidDataset('../datasets/20200602_154324_calib_mag/');
gyro_calib_sample = loadAndroidDataset('../datasets/calib_gyro_samsung/');
acc_calib_sample = loadAndroidDataset('../datasets/calib_ac_samsung/');

%% load magnetic north data

magnetic_north_sample = loadAndroidDataset('../datasets/magnetic_north/');

% Create context from location, date and coordinateSystem
magnetic = findMagneticField(location, date);

%%
% Calibrating Sensor Data 
[shs_sample, accSampled, gyrSampled, magSampled, dev_comp_attitude,calib_mag_north] = ...
    calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);


%% Step detection
[shs.steps, shs.data, shs.sd_components] = stepDetection(accSampled, 'data' , false);

% Step length estimation
shs.sl_components = timetable(shs.steps.data.Time);
shs.sl_components.period = [0; seconds(diff(shs.steps.data.Time))];
shs.sl_components.period(shs.sl_components.period == 0) = nan;
shs.sl_components.freq = 1./shs.sl_components.period;

male.k1 = 0.415;
male.k = 0.3139;

test_height = 1.78;

shs.steps.data.step_length = test_height.*male.k.*sqrt(shs.sl_components.freq);
shs.steps.data.step_length(1) = test_height.*male.k1;
shs.est_distance = sum(shs.steps.data.step_length);

%% Estimating orientation
prior_est = [dev_comp_attitude{1,:}]';

estimate1 = ExtendedKalmanFilter_series(prior_est, ...
                                accSampled, gyrSampled, magSampled, ...
                                calib_mag_north, false);
                          
[og_positions, step_orient] = plotTrajectory([estimate1.final_q{:,:}]',shs);
 %%
 close all
[dev_com_positions,~] = plotTrajectory([dev_comp_attitude{:,:}],shs);
%%
[og_positions, step_orient] = plotTrajectory([estimate1.final_q{:,:}]',shs);



