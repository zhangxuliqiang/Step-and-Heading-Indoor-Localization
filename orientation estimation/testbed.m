close all
clear variables
clc

file.directory = '../datasets/android/calib_mag_samsung/';
file.name = 'magnetometer';
file.time_unit = 1;

target.file = file;
target.dataSetProp = DataSetProp("Magnetometer","Time",(file.time_unit), ...
    ["mag_X","mag_Y","mag_Z"]);
mag_data.data = SSVFile2Timetable(target);

%%

file.directory = '../datasets/android/calib_acc_samsung/';
file.name = 'accelerometer';
file.time_unit = 1E-3;

target.file = file;
target.dataSetProp = DataSetProp("Magnetometer","Time",(file.time_unit), ...
    ["acc_X","acc_Y","acc_Z"]);
mag_data.name = file.name;
mag_data.data = SSVFile2Timetable(target);

%% Parameters %

name = 'around_the_block_samsung';
algorithmName = 'ekf';

% coordinateSystem can be 'enu' (East-North-Up) or 'ned' (North-East-Down)
coordinateSystem = 'enu';

% Location and date of data
location = struct('latitude', 52.0056634, 'longitude', 4.36677, 'altitude', 10);
date = struct('year', 2020, 'month', 05, 'day', 7);

%%% load Data %%%
% s = loadAndroidDataset(['../datasets/android/' name]);

s_cm = loadAndroidDataset('../datasets/android/calib_mag_samsung/');

%%
figure()
plot([mag_data.data.mag_X, mag_data.data.mag_Y, mag_data.data.mag_Z] == s_cm.rawIMU.magnetometer(:,2:4))