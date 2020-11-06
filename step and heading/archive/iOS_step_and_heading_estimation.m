clc
close all
clear variables

% folder_name = 'lopen2/';

file_name = ['okt 6 house session/lopen3/lopen3_iOS.csv'];

%% load Data %%%
shs_sample = loadiphoneDataset(['../datasets/' file_name]);

%% load calibration data
acc_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone acc calib/acc_calib_iOS.csv');
% mag_calib_sample = loadAndroidDataset('../datasets/okt 6 house session/android mag calib/android_calib2/');
gyro_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone gyro calib/gyro_calib_iOS.csv');
mag_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone mag calib/mag_calib_iOS.csv');

%% load magnetic north data

% magnetic_north_sample = loadAndroidDataset('../datasets/magnetic_north/');

magnetic_north_sample = loadiphoneDataset('../datasets/okt 6 house session/magnetic north iphone.csv');

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

file_name = 'finding_noise/';
noise_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone gyro calib/gyro_calib_iOS.csv');

[~, acc_noise, gyr_noise, mag_noise, ~,~] = ...
    calibrateSensors(noise_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);


variance.acc = max(var(acc_noise{:,:}));
variance.mag = max(var(mag_noise{:,:}));
variance.gyr = max(var(gyr_noise{:,:}));

%%

% prior_est = [dev_comp_attitude{4,:}]';
prior_est = [1,0,0,0]';

estimate1 = ExtendedKalmanFilter_series(prior_est, ...
                                accSampled, gyrSampled, magSampled, ...
                                calib_mag_north, variance, false);


clear target
target = timetable(estimate1.Time);
target.est = [estimate1.est{:,:}]';
euler_angles = timetable(estimate1.Time);
[euler_angles.yaw, euler_angles.pitch, euler_angles.roll] =  ...
    quat2angle([target.est ]);

[og_positions, step_orient] = plotTrajectory(target,shs);
%%

figure()
hold on
plot(euler_angles.Time,euler_angles.yaw)
plot(shs_sample.device_computed.attitude.Time,shs_sample.device_computed.attitude.Yaw)
hold off



temp_data = shs_sample.device_computed.attitude(shs.steps.data.Time,:);

[step_time,ia,~] = unique(temp_data.Time,'first');

step_orient = temp_data(ia,:);

step_orient.step_length = shs.steps.data.step_length;


positions = [];
prev_x = 0;
prev_y = 0;

for i = 1: height(shs.steps.data)
   pos.x = prev_x + cos(step_orient(i,:).Yaw).* shs.steps.data.step_length(i); 
   prev_x = pos.x;
   
   pos.y = prev_y + sin(step_orient(i,:).Yaw).* shs.steps.data.step_length(i); 
   prev_y = pos.y;  
   
   pos.time = seconds(shs.steps.data.Time(i));
   
   positions = [positions, pos];
end

%%
clear cov_euler
cov_quat = estimate1.P;
q = estimate1.est;
for i = 1:height(estimate1)
    cov_euler{i,1} = Gq(q{i})*cov_quat{i}*Gq(q{i})';
end
%
for i = 1:height(estimate1)
    diag_cov_euler(:,i) = diag(cov_euler{i});
    
end
figure();
stackedplot(diag_cov_euler');

%%
tester = estimate1(estimate1.corrected == 1,:).type;
unique(tester)
%%
covariance = sqrt(covariance);

[yaw, pitch, roll] =  ...
    quat2angle(target.est(:,:));

[cov_plus_yaw, cov_plus_pitch, cov_plus_roll] =  ...
    quat2angle(target.est(:,:) + covariance);

[cov_min_yaw, cov_min_pitch, cov_min_roll] =  ...
    quat2angle(target.est(:,:)-covariance);


figure
subplot(3,1,1)
hold on
plot(yaw)
plot(cov_plus_yaw)
plot(cov_min_yaw)
hold off
subplot(3,1,2)
hold on
plot(pitch)
plot(cov_plus_pitch)
plot(cov_min_pitch)
hold off
subplot(3,1,3)
hold on
plot(roll)
plot(cov_plus_roll)
plot(cov_min_roll)
hold off

%%
figure()
% plot(cov_plus_yaw)
subplot(3,1,1)
plot(yaw)
subplot(3,1,2)
plot(yaw-cov_plus_yaw)
subplot(3,1,3)
plot(yaw-cov_min_yaw)

%%
figure()
stackedplot([yaw,yaw-cov_plus_yaw,yaw-cov_min_yaw])

