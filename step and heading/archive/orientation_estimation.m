clc
% close all
clear variables

folder_name = 'walk_around_the_block_3_times/';

folder_name = 'walking_at_home/';

%%% load Data %%%
shs_sample = loadAndroidDataset(['../datasets/one plus nord/' folder_name]);

% load calibration data
% mag_calib_sample = loadAndroidDataset('../datasets/20200602_154324_calib_mag/');
% mag_calib_sample = loadAndroidDataset('../datasets/one plus nord/buiten_calib_mag/');
mag_calib_sample = loadAndroidDataset('../datasets/one plus nord/calib_mag/');
gyro_calib_sample = loadAndroidDataset('../datasets/one plus nord/calib_gyr/');
acc_calib_sample = loadAndroidDataset('../datasets/one plus nord/calib_acc/');

% load magnetic north data

magnetic_north_sample = loadAndroidDataset('../datasets/one plus nord/magnetic_north_buiten/');
% magnetic_north_sample = loadAndroidDataset('../datasets/one plus nord/indoor_magnetic_north/');
% magnetic_north_sample = loadAndroidDataset('../datasets/okt 6 house session/indoor magnetic north/');

% Calibrating Sensor Data 
[~, accSampled, gyrSampled, magSampled, dev_comp_attitude,calib_mag_north] = ...
    calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);

 %Estimating orientation
noise_sample = loadAndroidDataset('../datasets/one plus nord/noise/');

[~, acc_noise, gyr_noise, mag_noise, ~,~] = ...
    calibrateSensors(noise_sample, mag_calib_sample, acc_calib_sample, ...
                     gyro_calib_sample,magnetic_north_sample);

variance.acc = max(var(acc_noise{:,:}));
variance.mag = max(var(mag_noise{:,:}));
variance.gyr = max(var(gyr_noise{:,:}));

%  prior_est = [dev_comp_attitude{4,:}]';
prior_est = [1,0,0,0]';

ekf_estimate = ExtendedKalmanFilter_series(prior_est, ...
                                accSampled, gyrSampled, magSampled, ...
                                calib_mag_north, variance, false);

mekf_estimate = MultiplicativeExtendedKalmanFilter_series(prior_est, ...
                                accSampled, gyrSampled, magSampled, ...
                                calib_mag_north, variance, false);

%%
clear target
mekf = timetable(mekf_estimate.Time);
mekf.est = [mekf_estimate.est{:,:}]';

mekf_euler_angles = timetable(mekf_estimate.Time);
[mekf_euler_angles.yaw, mekf_euler_angles.pitch, mekf_euler_angles.roll] =  ...
    quat2angle([mekf.est]);

ekf = timetable(ekf_estimate.Time);
ekf.est = [ekf_estimate.est{:,:}]';

ekf_euler_angles = timetable(ekf_estimate.Time);
[ekf_euler_angles.yaw, ekf_euler_angles.pitch, ekf_euler_angles.roll] =  ...
    quat2angle([ekf.est]);

phone_estimate = timetable(shs_sample.device_computed.attitude.Time);
[phone_estimate.yaw, phone_estimate.pitch, phone_estimate.roll] =  ...
    quat2angle([shs_sample.device_computed.attitude{:,:}]);

figure
subplot(3,3,1)
plot(ekf_euler_angles.Time,ekf_euler_angles.yaw)
title('EKF: yaw')
subplot(3,3,4)
plot(mekf_euler_angles.Time,mekf_euler_angles.yaw)
title('MEKF: yaw')
subplot(3,3,7)
plot(phone_estimate.Time,phone_estimate.yaw)
title('Phone Estimate: yaw')

subplot(3,3,2)
plot(ekf_euler_angles.Time,ekf_euler_angles.roll)
title('EKF: roll')
subplot(3,3,5)
plot(mekf_euler_angles.Time,mekf_euler_angles.roll)
title('MEKF: roll')
subplot(3,3,8)
plot(phone_estimate.Time,phone_estimate.roll)
title('Phone Estimate: roll')

subplot(3,3,3)
plot(ekf_euler_angles.Time,ekf_euler_angles.pitch)
title('EKF: pitch')
subplot(3,3,6)
plot(mekf_euler_angles.Time,mekf_euler_angles.pitch)
title('MEKF: pitch')
subplot(3,3,9)
plot(phone_estimate.Time,phone_estimate.pitch)
title('Phone Estimate: pitch')

% figure
% subplot(2,1,1)
% plot(ekf_euler_angles.Time,mekf_euler_angles.yaw - ekf_euler_angles.yaw)
% title('EKF')
% subplot(2,1,2)
% plot(mekf_euler_angles.Time,phone_estimate.yaw - mekf_euler_angles.yaw)
% title('MEKF')

%%
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

clear target
target = timetable(ekf_estimate.Time);
target.est = [ekf_estimate.est{:,:}]';

[og_positions0, step_orient0] = plotTrajectory(target,shs);

target = timetable(mekf_estimate.Time);
target.est = [mekf_estimate.est{:,:}]';

[og_positions1, step_orient1] = plotTrajectory(target,shs);



%%
clear cov_euler
cov_quat = ekf_estimate.P;
q = ekf_estimate.est;

for i = 1:height(ekf_estimate)
    cov_euler{i,1} = Gq(q{i})*cov_quat{i}*Gq(q{i})';
end
%
for i = 1:height(estimate1)
    diag_cov_euler(:,i) = diag(cov_euler{i});
    
end
figure();
stackedplot(diag_cov_euler');