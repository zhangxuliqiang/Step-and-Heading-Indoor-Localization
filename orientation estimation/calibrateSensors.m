function [calib_data, dev_comp_attitude, variance] = ...
    calibrateSensors(shs_sample,calib_samples,noise_sample, magnetic_north_sample)

disp('Calibrating sensor data')

 mag_calib_sample = calib_samples.mag_calib_sample ;
 acc_calib_sample = calib_samples.acc_calib_sample;
 gyr_calib_sample = calib_samples.gyr_calib_sample ;

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

%% Gyroscope Calibration
gyr_bias = mean(gyr_calib_sample.raw_imu.gyroscope{:,:});
calib_gyr = shs_sample.raw_imu.gyroscope{:,1:3}-gyr_bias;
calib_gyr_data = timetable(shs_sample.raw_imu.gyroscope.Time);
calib_gyr_data.X = calib_gyr(:,1);
calib_gyr_data.Y = calib_gyr(:,2);
calib_gyr_data.Z = calib_gyr(:,3);

%% Populate fuction output
calib_data.acc_data = calib_acc_data;
calib_data.gyr_data = calib_gyr_data;
calib_data.mag_data = calib_mag_data;
calib_data.mag_north_data = calib_mag_north_data;

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

end