close all
clear variables
clc

% Parameters %
dataset_directory = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/';
folder_name = 'around_the_block_samsung/';

% Location and date of data
location = struct('latitude', 52.0056634, 'longitude', 4.36677, 'altitude', 10);
date = struct('year', 2020, 'month', 05, 'day', 7);

%%% load Data %%%
shs_sample = loadAndroidDataset([dataset_directory folder_name]);
%
mag_calib_sample = loadAndroidDataset([dataset_directory 'calib_mag_samsung/']);
gyro_calib_sample = loadAndroidDataset([dataset_directory 'calib_gyro_samsung/']);
acc_calib_sample = loadAndroidDataset([dataset_directory 'calib_ac_samsung/']);

% Create context from location, date and coordinateSystem
magnetic = findMagneticField(location, date);

%%
[shs_sample, accSampled, gyrSampled, magSampled, dev_comp_attitude] = ...
    calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, gyro_calib_sample,magnetic);

%%
tic
estimate_att = Bart_EKF(seconds(accSampled.Time), ...
                        accSampled{:,:}, gyrSampled{:,:}, magSampled{:,:},...
                        magnetic);
toc
%%
[euler_angles(:, 1), euler_angles(:, 2), euler_angles(:, 3)] = quat2angle(estimate_att);

%%
[computed_euler_angles(:, 1), computed_euler_angles(:, 2), computed_euler_angles(:, 3)] = ...
    quat2angle(shs_sample.device_computed.attitude{:,:});

% figure()
% stackedplot(euler_angles)
% title('BART EKF')

figure()
stackedplot(computed_euler_angles)
title('BART EKF')
