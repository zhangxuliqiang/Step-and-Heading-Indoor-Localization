clc
close all
clear variables

PLOTTER = true;

% folder_name = 'walk_around_the_block_3_times/';
% 
% folder_name = 'walking_at_home/';

%% ================== load Data ======================

file_name = ['okt 6 house session/lopen3/lopen3_iOS.csv'];

shs_sample = loadiphoneDataset(['../datasets/' file_name]);

% load calibration data
acc_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone acc calib/acc_calib_iOS.csv');
% mag_calib_sample = loadAndroidDataset('../datasets/okt 6 house session/android mag calib/android_calib2/');
gyro_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone gyro calib/gyro_calib_iOS.csv');
mag_calib_sample = loadiphoneDataset('../datasets/okt 6 house session/iphone calib/iphone mag calib/mag_calib_iOS.csv');

%% -------- PLOTTER: shs raw imu sample
target = shs_sample;
if PLOTTER
    figure
    sgtitle('SHS sample raw IMU data')
        subplot(3,1,1)    
        stackedplot(target.raw_imu.accelerometer)
        title('raw imu data - accelerometer')
        subplot(3,1,2)    
        stackedplot(target.raw_imu.gyroscope)
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(target.raw_imu.magnetometer)
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end

%% load magnetic north data

% magnetic_north_sample = loadAndroidDataset('datasets/magnetic_north/');
magnetic_north_sample = loadiphoneDataset('../datasets/okt 6 house session/magnetic north iphone.csv');
%% -------- PLOTTER: shs magnetic north raw data 
target = magnetic_north_sample;
if PLOTTER
    figure
    sgtitle('magnetic north sample raw IMU data')
        subplot(3,1,1)    
        stackedplot(target.raw_imu.accelerometer)
        title('raw imu data - accelerometer')
        subplot(3,1,2)    
        stackedplot(target.raw_imu.gyroscope)
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(target.raw_imu.magnetometer)
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end

%% ================= CALIBRATING SENSOR DATA ==========================
%% Magnetometer calibration
clc
disp('Calibrating magnetometer sensor data')

mag_raw = mag_calib_sample.raw_imu.magnetometer{:,:};

N=size(mag_raw,1);
M=ones(N,13);
for i=1:N
    M(i,1:9)=kron(mag_raw(i,:),mag_raw(i,:));
    M(i,10:12)=mag_raw(i,:);
end

cvx_begin
    variable A(3,3)
    variable b(3,1)
    variable c(1,1)
    minimize(norm( M * [vec(A) ; b ; c] , 2 ) )
    subject to
    trace(A) == 1
    A-0.0001*eye(3) == semidefinite(3)
cvx_end

invDT_invD = inv(0.25 * b' *inv(A) * b - c) *A;
invD = chol(invDT_invD);
bias = -0.5* inv(A) * b ;

mag_bias = bias;
mag_invD = invD;

calib_mag = mag_invD*(shs_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
calib_mag_data = timetable(shs_sample.raw_imu.magnetometer.Time);
calib_mag_data.X = calib_mag(1,:)';
calib_mag_data.Y = calib_mag(2,:)';
calib_mag_data.Z = calib_mag(3,:)';

% Magnetic North Calibration
calib_mag_north = mag_invD*(magnetic_north_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
calib_mag_north_data = timetable(magnetic_north_sample.raw_imu.magnetometer.Time);
calib_mag_north_data.X = calib_mag_north(1,:)';
calib_mag_north_data.Y = calib_mag_north(2,:)';
calib_mag_north_data.Z = calib_mag_north(3,:)';

%% -------- PLOTTER: magnetometer calibration
close all
% magnetometer calib data calibration
calib_mag_calib = mag_invD*(mag_raw'- mag_bias);

figure()
hold on
scatter3(calib_mag_calib(1,:),calib_mag_calib(2,:),calib_mag_calib(3,:),'r')
scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:),'b')
hold off
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
legend('calibrated calibration data','calibrated SHS data')
title('Calibrated Magnetometer Data')

%% Accelerometer Calibration

acc_raw = acc_calib_sample.raw_imu.accelerometer{:,:};

N = size(acc_raw,1);
M=ones(N,13);

for i=1:N
    M(i,1:9)=kron(acc_raw(i,:),acc_raw(i,:));
    M(i,10:12)=acc_raw(i,:);
end

cvx_begin
    variable A(3,3)
    variable b(3,1)
    variable c(1,1)
    minimize( norm( M * [vec(A) ; b ; c] , 2 ) )
    subject to
    trace(A) == 1
    A-0.0001*eye(3) == semidefinite(3)
cvx_end

invDT_invD = inv(0.25 * ( b' / A * b ) - c) * A;
acc_invD = chol(invDT_invD);
acc_bias = -0.5* ( A \ b );

calib_acc = acc_invD*(shs_sample.raw_imu.accelerometer{:,1:3}'- acc_bias);
calib_acc = calib_acc.* 9.81;
calib_acc_data = timetable(shs_sample.raw_imu.accelerometer.Time);
calib_acc_data.X = calib_acc(1,:)';
calib_acc_data.Y = calib_acc(2,:)';
calib_acc_data.Z = calib_acc(3,:)';

%% -------- PLOTTER: accelerometer calibration
close all
% magnetometer calib data calibration
calib_acc_calib = acc_invD*(acc_raw'- acc_bias);

figure()
hold on
scatter3(calib_acc_calib(1,:),calib_acc_calib(2,:),calib_acc_calib(3,:),'r')
scatter3(calib_acc(1,:),calib_acc(2,:),calib_acc(3,:),'b')
hold off
% xlim([-1,1])
% ylim([-1,1])
% zlim([-1,1])
legend('calibrated calibration data','calibrated SHS data')
title('Calibrated Accelerometer Data')

%% Gyroscope Calibration
gyr_bias = mean(gyro_calib_sample.raw_imu.gyroscope{:,:});
calib_gyr = shs_sample.raw_imu.gyroscope{:,1:3}-gyr_bias;
calib_gyr_data = timetable(shs_sample.raw_imu.gyroscope.Time);
calib_gyr_data.X = calib_gyr(:,1);
calib_gyr_data.Y = calib_gyr(:,2);
calib_gyr_data.Z = calib_gyr(:,3);

% Align sensors data
acc_data = calib_acc_data;
gyro_data = calib_gyr_data;
mag_data = calib_mag_data;
dev_comp_attitude = shs_sample.device_computed.attitude;


%% finding variance orientation

folder_name = 'finding_noise/';
noise_sample = loadAndroidDataset(['datasets/' folder_name]);

noise_calib_mag = mag_invD*(noise_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
noise_mag_data = timetable(noise_sample.raw_imu.magnetometer.Time);
noise_mag_data.X = noise_calib_mag(1,:)';
noise_mag_data.Y = noise_calib_mag(2,:)';
noise_mag_data.Z = noise_calib_mag(3,:)';

% Accelerometer Calibration

noise_calib_acc = acc_invD*(noise_sample.raw_imu.accelerometer{:,1:3}'- acc_bias);
noise_calib_acc = noise_calib_acc.* 9.81;
noise_acc_data = timetable(noise_sample.raw_imu.accelerometer.Time);
noise_acc_data.X = noise_calib_acc(1,:)';
noise_acc_data.Y = noise_calib_acc(2,:)';
noise_acc_data.Z = noise_calib_acc(3,:)';

% Gyroscope Calibration
noise_calib_gyr = noise_sample.raw_imu.gyroscope{:,1:3}-gyr_bias;
noise_calib_gyr_data = timetable(noise_sample.raw_imu.gyroscope.Time);
noise_calib_gyr_data.X = noise_calib_gyr(:,1);
noise_calib_gyr_data.Y = noise_calib_gyr(:,2);
noise_calib_gyr_data.Z = noise_calib_gyr(:,3);  
                 
variance.acc = max(var(noise_acc_data{:,:}));
variance.mag = max(var(noise_mag_data{:,:}));
variance.gyr = max(var(noise_calib_gyr_data{:,:}));

%%

%  prior_est = [dev_comp_attitude{4,:}]';
prior_est = [1,0,0,0]';

ekf_estimate = ExtendedKalmanFilter_series(prior_est, ...
                                acc_data, gyro_data, mag_data, ...
                                calib_mag_north_data, variance, false);

% estimate1 = MultiplicativeExtendedKalmanFilter_series(prior_est, ...
%                                 accSampled, gyrSampled, magSampled, ...
%                                 calib_mag_north, variance, false);
% 
% %%

% clear target
% mekf = timetable(mekf_estimate.Time);
% mekf.est = [mekf_estimate.est{:,:}]';
% 
% mekf_euler_angles = timetable(mekf_estimate.Time);
% [mekf_euler_angles.yaw, mekf_euler_angles.pitch, mekf_euler_angles.roll] =  ...
%     quat2angle([mekf.est]);
%%
ekf = timetable(ekf_estimate.Time);
ekf.est = [ekf_estimate.est{:,:}]';

ekf_euler_angles = timetable(ekf_estimate.Time);
[ekf_euler_angles.yaw, ekf_euler_angles.pitch, ekf_euler_angles.roll] =  ...
    quat2angle([ekf.est]);

phone_estimate = timetable(shs_sample.device_computed.attitude.Time);
[phone_estimate.yaw, phone_estimate.pitch, phone_estimate.roll] =  ...
    quat2angle([shs_sample.device_computed.attitude{:,:}]);

figure
subplot(2,1,1)
plot(ekf_euler_angles.Time,ekf_euler_angles.yaw)
subplot(2,1,2)
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

%%

[shs.steps, shs.data, shs.sd_components] = stepDetection(acc_data, 'data' , false);

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
%
target = timetable(ekf_estimate.Time);
target.est = [ekf_estimate.est{:,:}]';

[og_positions1, step_orient1] = plotTrajectory(target,shs);
