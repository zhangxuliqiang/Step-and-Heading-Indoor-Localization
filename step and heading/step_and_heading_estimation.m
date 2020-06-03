clc
close all
% clear variables

folder_name = '3_times_around_the_block/';

% Location and date of data
location = struct('latitude', 52.0056634, 'longitude', 4.36677, 'altitude', 10);
date = struct('year', 2020, 'month', 05, 'day', 7);

%%% load Data %%%
shs_sample = loadAndroidDataset(['../datasets/' folder_name]);
%
mag_calib_sample = loadAndroidDataset('../datasets/20200602_154324_calib_mag/');
gyro_calib_sample = loadAndroidDataset('../datasets/calib_gyro_samsung/');
acc_calib_sample = loadAndroidDataset('../datasets/calib_ac_samsung/');

% Create context from location, date and coordinateSystem
magnetic = findMagneticField(location, date);

% Calibrating Sensor Data 
[shs_sample, accSampled, gyrSampled, magSampled, dev_comp_attitude] = calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, gyro_calib_sample,magnetic);

%% Estimating orientation

[estimate_att,debug] = Bart_EKF(seconds(accSampled.Time), accSampled{:,:}, gyrSampled{:,:}, magSampled{:,:}, magnetic);
%%
clc
estimate = ExtendedKalmanFilter(accSampled, gyrSampled, magSampled, magnetic, true);
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


%%
euler_angles = timetable(shs.data.Time);
[euler_angles.yaw, euler_angles.pitch, euler_angles.roll] = quat2angle([estimate.post_mag_est{:,:}]');
% [euler_angles.yaw, euler_angles.pitch, euler_angles.roll] = quat2angle(estimate_att);

positions = [];
prev_x = 0;
prev_y = 0;

euler_angles.yaw = euler_angles.yaw + pi/2;

step_orient = euler_angles(shs.steps.data.Time, :);

for i = 1: height(step_orient)
   pos.x = prev_x + cos(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_x = pos.x;
   
   pos.y = prev_y + sin(step_orient(i,:).yaw).* shs.steps.data.step_length(i); 
   prev_y = pos.y;   
   
   positions = [positions, pos];
end


figure()
hold on
plot3([positions.x],[positions.y],step_orient.Time)
hold off
xlabel('X position from initial (meters)')
ylabel('Y position from initial (meters)')
xlim([-150 50])
ylim([-50 150])
title('step and heading system walking around the block')
axis square
