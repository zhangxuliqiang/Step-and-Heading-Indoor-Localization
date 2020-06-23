function [shs_sample, accSampled, gyrSampled, magSampled, dev_comp_attitude,calib_mag_north_data] = ...
    calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, gyro_calib_sample,mag_north_sample)

disp('Calibrating sensor data')

% Magnetometer Calibration
[mag_invD, mag_bias]=magCalib(mag_calib_sample.raw_imu.magnetometer{:,1:3});
calib_mag = mag_invD*(shs_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
calib_mag_data = timetable(shs_sample.raw_imu.magnetometer.Time);
calib_mag_data.X = calib_mag(1,:)';
calib_mag_data.Y = calib_mag(2,:)';
calib_mag_data.Z = calib_mag(3,:)';

% Magnetic North Calibration
calib_mag_north = mag_invD*(mag_north_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
calib_mag_north_data = timetable(mag_north_sample.raw_imu.magnetometer.Time);
calib_mag_north_data.X = calib_mag_north(1,:)';
calib_mag_north_data.Y = calib_mag_north(2,:)';
calib_mag_north_data.Z = calib_mag_north(3,:)';

% Accelerometer Calibration
staticValues = findStaticRegions(acc_calib_sample.raw_imu);

[acc_invD,acc_bias] = accCalib(staticValues);
calib_acc = acc_invD*(shs_sample.raw_imu.accelerometer{:,1:3}'- acc_bias);
calib_acc = calib_acc.* 9.81;
calib_acc_data = timetable(shs_sample.raw_imu.accelerometer.Time);
calib_acc_data.X = calib_acc(1,:)';
calib_acc_data.Y = calib_acc(2,:)';
calib_acc_data.Z = calib_acc(3,:)';

% Gyroscope Calibration
gyr_bias = gyroCalib(gyro_calib_sample.raw_imu);
calib_gyr = shs_sample.raw_imu.gyroscope{:,1:3}-gyr_bias;
calib_gyr_data = timetable(shs_sample.raw_imu.gyroscope.Time);
calib_gyr_data.X = calib_gyr(:,1);
calib_gyr_data.Y = calib_gyr(:,2);
calib_gyr_data.Z = calib_gyr(:,3);

% Align sensors data
acc = calib_acc_data;
gyr = calib_gyr_data;
mag = calib_mag_data;

firstTimestamp = max([acc.Time(1), gyr.Time(1), mag.Time(1)]);
lastTimestamp = min([acc.Time(end), gyr.Time(end), mag.Time(end)]);

dT = seconds(1/100);
timestamp = firstTimestamp:dT:lastTimestamp;

accSampled = acc;
gyrSampled = gyr;
magSampled = mag;
dev_comp_attitude = shs_sample.device_computed.attitude;

end