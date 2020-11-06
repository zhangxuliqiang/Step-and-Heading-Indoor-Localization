clc
close all
clear variables

PLOTTER = true;
data_set_name = 'lopen1.2';

%% ================== load Data ======================
if strcmp(data_set_name, 'lopen1.2' )
    shs_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/lopen1_2/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/marie testing/lopen1.2/noise/');
end

%%

if strcmp(data_set_name, 'lopen1.1' )
    shs_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/lopen1_1/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/marie testing/lopen1.1/noise/');
end

%%

if strcmp(data_set_name, 'inside_walking_record_ns_marie' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/walking_around_door_record/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
end

%%

if strcmp(data_set_name, 'inside_walking_ns_marie' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/binnen_lopen_1_geen_smartwatch/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
end
%%
if strcmp(data_set_name, 'inside_stationary_marie' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/stationary_test/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/20 October stationary/noise/');
end


%% walking outside dataset

if strcmp(data_set_name, 'outside' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/13 October/two_times_around_the_block/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_gyr/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/13 October/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/13 October/magnetic_north_buiten/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/13 October/magnetic_north_buiten/');
end
%%
if strcmp(data_set_name, 'stationary' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/15 October/stationary_test/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_gyr/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/15 October/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/15 October/noise/');
end
%%
if strcmp(data_set_name, 'inside' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/walking_inside/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_gyr/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/noise/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/15 October walking/noise/');
end
%%
if strcmp(data_set_name, 'inside tracked' )
    shs_sample = loadAndroidDataset('datasets/one plus nord/17 October/walking_around_inside/');
    
    % load calibration data
    calib_samples.mag_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/calib_mag/');
    calib_samples.gyr_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
    calib_samples.acc_calib_sample = loadAndroidDataset('datasets/one plus nord/17 October/calib_acc/');
    
    % load magnetic north data
    magnetic_north_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
    
    % load noise sample
    noise_sample = loadAndroidDataset('datasets/one plus nord/17 October/laying_on_the_floor/');
end

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

mag_raw = calib_samples.mag_calib_sample.raw_imu.magnetometer{:,:};

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
% magnetometer calib data calibration
calib_mag_calib = mag_invD*(mag_raw'- mag_bias);

figure()
hold on
scatter3(calib_mag_calib(1,:),calib_mag_calib(2,:),calib_mag_calib(3,:),'r')
scatter3(calib_mag(1,:),calib_mag(2,:),calib_mag(3,:),'b')
scatter3(calib_mag_north(1,:),calib_mag_north(2,:),calib_mag_north(3,:),'g')
hold off
xlim([-1,1])
ylim([-1,1])
zlim([-1,1])
legend('calibrated calibration data','calibrated SHS data','calibrated magnetic north')
title('Calibrated Magnetometer Data')

%% Accelerometer Calibration

acc_raw = calib_samples.acc_calib_sample.raw_imu.accelerometer{:,:};

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

% magnetometer calib data calibration
calib_acc_calib = 9.81*acc_invD*(acc_raw'- acc_bias);

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
gyr_bias = mean(calib_samples.gyr_calib_sample.raw_imu.gyroscope{:,:});
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
noise_gyro_data = timetable(noise_sample.raw_imu.gyroscope.Time);
noise_gyro_data.X = noise_calib_gyr(:,1);
noise_gyro_data.Y = noise_calib_gyr(:,2);
noise_gyro_data.Z = noise_calib_gyr(:,3);  
                 
variance.acc = max(var(noise_acc_data{:,:}));
variance.mag = max(var(noise_mag_data{:,:}));
variance.gyr = max(var(noise_gyro_data{:,:}));

%% -------- PLOTTER: noise realizations

if PLOTTER
    figure
    sgtitle('noise realization')
        subplot(3,1,1)    
        stackedplot(noise_acc_data)
        title('raw imu data - accelerometer')
        subplot(3,1,2)    
        stackedplot(noise_gyro_data)
        title('raw imu data - gyroscope')
        subplot(3,1,3)
        stackedplot(noise_mag_data)
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end

%% Extended Kalman Filter

%  prior estimate
prior_est = [1,0,0,0]';
% prior_est = [dev_comp_attitude{4,:}]';

est = prior_est;

Types = categorical({'GYR','ACC', 'MAG'});

gyro_data.Type(:) = Types(1);
acc_data.Type(:) = Types(2);
mag_data.Type(:) = Types(3);

combined_raw = [acc_data; mag_data; gyro_data];
combined_raw = sortrows(combined_raw);

P = 1 * eye(4);

calAcc_R = variance.acc * eye(3);

calGyr_R = variance.gyr * eye(3);

calMag_R = variance.mag * eye(3);

combined = [combined_raw.X, combined_raw.Y, combined_raw.Z]';
type = [combined_raw.Type];

dT = [0; seconds(diff(gyro_data.Time))];
Time = [seconds(combined_raw.Time)];

    
estimate = repmat(struct('Time', nan, ...
                         'est',nan, ...
                         'P', nan,...
                         'error', nan,...
                         'type', nan,...
                         'corrected', nan),...
                         height(combined_raw), 1 );


g = [0; 0; -9.81];

mag_vector = mean(calib_mag_north_data{:,:});
mag_field =  transpose(mag_vector./norm(mag_vector));

counter = 0;
progress_before= 0;
for index = 1:1:height(combined_raw)
    
    progress = round(index/height(combined_raw)*100,2);
    if mod(progress,10) == 0  && round(progress) ~= 0
        if progress~=progress_before
        disp(['percentage complete: ',  num2str(progress)]);
        progress_before = progress;
        end
    end
    
    y = combined(:,index) ;
    error = nan;
    corrected = 0;
    switch type(index)
        
        case Types(1)
            counter = counter +1;
            
            % -------------  MOTION UPDATE -----------------------
            F = eye(4) + 0.5*(dT(counter)* Somega(y));
            Gu= dT(counter)./ 2 *Sq(est);
            est = F* est; 
            J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
            est = est / norm(est);
            P = F*P*F' + Gu*calGyr_R*Gu';
%             P = J*P*J';
    % -------------- MEASUREMENT UPDATES ------------------
       
            
        case Types(2)
            % Accelerometer measurement update
            dRdq_acc = dRqdq(est);
            H_acc = [-dRdq_acc(:,:,1)'*g,...
                     -dRdq_acc(:,:,2)'*g,...
                     -dRdq_acc(:,:,3)'*g,...
                     -dRdq_acc(:,:,4)'*g];

            H = [H_acc];

            y_hat = [-quat2rotmat(est)' * g];

            R = [calAcc_R];

            error = y - y_hat;
            
            if norm(y) < 10.0 && norm(y) > 9.6
                [est,P] = measUpdate(est,P,error,H,R);            
                J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                est = est/norm(est); 
                corrected = 1;
%                 P = J*P*J';
            end
            
         case Types(3)
             % magnetometer measurement update
            dRdq_mag = dRqdq(est);
            H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
                     dRdq_mag(:,:,2)'*mag_field, ...
                     dRdq_mag(:,:,3)'*mag_field, ...
                     dRdq_mag(:,:,4)'*mag_field];

            H = [ H_mag];

            y_hat = [ quat2rotmat(est)' * mag_field];

            R = [ calMag_R];

            error = y - y_hat;
            
             if norm(y) < 1.2 && norm(y) > 0.8
                 [est,P] = measUpdate(est,P,error,H,R);
                 J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                 est = est/norm(est);
                 corrected = 1;
%                  P = J*P*J';
             end

    end
        
%     --------------- SAVING ESTIMATE COMPONENTS ---------
    x.Time =Time(index);
    x.est = est;
    x.P = P;
    x.error = error;
    x.type = type(index);
    x.corrected = corrected;

    
    estimate(index) = x;
    
end

estimate = struct2table(estimate);
estimate.Time = seconds(estimate.Time);
ekf_estimate = table2timetable(estimate);
    
%% Error plotting
ekf_acc_errors  = ekf_estimate(ekf_estimate.type=='ACC',:);
ekf_mag_errors  = ekf_estimate(ekf_estimate.type=='MAG',:);

if PLOTTER
    figure
    sgtitle('EKF errors')
        subplot(2,1,1)    
        stackedplot([ekf_acc_errors.error{:,:}]','DisplayLabels',{'x' 'y' 'z'})
        title('raw imu data - accelerometer')
        subplot(2,1,2)
        stackedplot([ekf_mag_errors.error{:,:}]','DisplayLabels',{'x' 'y' 'z'})
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end

%% multiplicative extended kalman filter

mekf_estimate = MultiplicativeExtendedKalmanFilter_series(prior_est, ...
                                acc_data, gyro_data, mag_data, ...
                                calib_mag_north_data, variance, false);

%% Error plotting
mekf_acc_errors  = mekf_estimate(ekf_estimate.type=='ACC',:);
mekf_mag_errors  = mekf_estimate(ekf_estimate.type=='MAG',:);

if PLOTTER
    figure
    sgtitle('EKF errors')
        subplot(2,1,1)    
        stackedplot([mekf_acc_errors.error{:,:}]')
        title('raw imu data - accelerometer')
        subplot(2,1,2)
        stackedplot([mekf_mag_errors.error{:,:}]')
        title('raw imu data - magnetometer')
    set(gcf,'position',[ 1434    138    548   489])
end
%%
mekf_euler_angles = timetable(mekf_estimate.Time);
euler = q2euler([mekf_estimate.est{:,:}]);

mekf_euler_angles.yaw = euler(1,:)';
mekf_euler_angles.pitch = euler(2,:)';
mekf_euler_angles.roll = euler(3,:)';

%% 
ekf_euler_angles = timetable(ekf_estimate.Time);
euler = q2euler([ekf_estimate.est{:,:}]);

ekf_euler_angles.yaw = euler(1,:)';
ekf_euler_angles.pitch = euler(2,:)';
ekf_euler_angles.roll = euler(3,:)';
% 
phone_estimate = timetable(shs_sample.device_computed.attitude.Time);
euler = q2euler([shs_sample.device_computed.attitude{:,:}']);

phone_estimate.yaw = euler(1,:)';
phone_estimate.pitch = euler(2,:)';
phone_estimate.roll = euler(3,:)';

figure
subplot(2,1,1)
stackedplot(ekf_euler_angles)
title('EKF estimate')
subplot(2,1,2)
stackedplot(phone_estimate)
title('Phone Estimate')
set(gcf,'position',[ 1434    138    548   489])


% figure
% subplot(3,1,1)
% stackedplot(mekf_euler_angles)
% title('EKF estimate')
% subplot(3,1,2)
% stackedplot(ekf_euler_angles)
% title('EKF estimate')
% subplot(3,1,3)
% stackedplot(phone_estimate)
% title('Phone Estimate')
% set(gcf,'position',[ 1434    138    548   489])
% 
% subplot(3,3,2)
% plot(ekf_euler_angles.Time,ekf_euler_angles.roll)
% title('EKF: roll')
% subplot(3,3,5)
% plot(mekf_euler_angles.Time,mekf_euler_angles.roll)
% title('MEKF: roll')
% subplot(3,3,8)
% plot(phone_estimate.Time,phone_estimate.roll)
% title('Phone Estimate: roll')
% 
% subplot(3,3,3)
% plot(ekf_euler_angles.Time,ekf_euler_angles.pitch)
% title('EKF: pitch')
% subplot(3,3,6)
% plot(mekf_euler_angles.Time,mekf_euler_angles.pitch)
% title('MEKF: pitch')
% subplot(3,3,9)
% plot(phone_estimate.Time,phone_estimate.pitch)
% title('Phone Estimate: pitch')

%%
figure
subplot(3,1,1)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.yaw)
plot(phone_estimate.Time, phone_estimate.yaw)
hold off
legend('ekf','phone estimate')
title('yaw')
subplot(3,1,2)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.roll)
plot(phone_estimate.Time, phone_estimate.roll)
hold off
legend('ekf','phone estimate')
title('roll')
subplot(3,1,3)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.pitch)
plot(phone_estimate.Time, phone_estimate.pitch)
hold off
legend('ekf','phone estimate')
title('pitch')
sgtitle([ data_set_name ' example'])
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
door_handle_use = ReferenceFile2Timetable('datasets/marie testing/lopen1.2/lopen1_2/references.txt');

target = timetable(ekf_estimate.Time);
target.est = [ekf_estimate.est{:,:}]';
[og_positions1, step_orient1] = plotTrajectory(target,shs,door_handle_use);

clear target
target = timetable(shs_sample.device_computed.attitude.Time);
target.est = shs_sample.device_computed.attitude{:,:};
target = retime(target,unique(ekf_estimate.Time),'linear');
[og_positions1, step_orient1] = plotTrajectory(target,shs,door_handle_use);

%% Functions

function [est,P] = measUpdate(est,P,error,H,R)
    % EKF measurement update
    S = H*P*H' + R;
    K = (P*H') / S;
    P = P - K*S*K'; 
    est = est + K*error;
end

function R = quat2rotmat(q)
    % Convert a quaternion to a rotation matrix
    q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
    R = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
        2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
        2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2) - 1];
end

function dRdq = dRqdq(q)
    % Derivative of a rotation matrix wrt a quaternion
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   dRdq(:,:,1) = 2* [2*q0   -q3    q2;
              q3  2*q0   -q1;
             -q2    q1  2*q0];
   dRdq(:,:,2) = 2* [2*q1    q2    q3;
              q2     0   -q0;
              q3    q0     0];
   dRdq(:,:,3) = 2* [   0    q1    q0;
              q1  2*q2    q3;
             -q0    q3     0];
   dRdq(:,:,4) = 2* [   0   -q0    q1;
              q0     0    q2;
              q1    q2  2*q3];
end

function S=Somega(w)
% The matrix S(omega) defined in (13.11b)
   wx=w(1);   wy=w(2);   wz=w(3);
   S=[ 0  -wx  -wy  -wz;
      wx    0   wz  -wy;
      wy  -wz    0   wx;
      wz   wy  -wx    0];
end

function S=Sq(q)
% The matrix S(q) defined in (13.11c)
   q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
   S=[-q1 -q2 -q3;
       q0 -q3  q2;
       q3  q0 -q1;
      -q2  q1  q0];
end


function euler = q2euler(q)
% Q2EULER  Convert quaternions to Euler angles
% euler = q2euler(q)
% q is a quaternion in columns (4xN)
% euler = [yaw(z) ; pitch(x) ; roll(y)]
  q = [q(1,:);q(3,:);q(2,:);q(4,:)];
  euler = zeros(3, size(q, 2));

  xzpwy = q(2, :).*q(4, :) + q(1, :).*q(3, :);

	IN = xzpwy+sqrt(eps)>0.5;  % Handle the north pole
  euler(1, IN) = 2*atan2(q(2, IN), q(1, IN));
  IS = xzpwy-sqrt(eps)<-0.5;  % Handle the south pole
  euler(1, IS) = -2*atan2(q(2, IS), q(1, IS));

  I = ~(IN | IS);  % Handle the default case

  euler(1, I) = atan2(2*(q(2, I).*q(3, I) - q(1, I).*q(4, I)),...
                      1-2*(q(3, I).^2 + q(4, I).^2));


  euler(3, I) = atan2(2*(q(3, I).*q(4, I) - q(1, I).*q(2, I)),...
                      1-2*(q(2, I).^2 + q(3, I).^2));

  euler(2, :) = -asin(2*xzpwy);

  euler = mod(euler+pi, 2*pi) - pi;
end
