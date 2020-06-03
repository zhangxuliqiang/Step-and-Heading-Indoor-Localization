function [shs_sample, accSampled, gyrSampled, magSampled,dev_comp_attitude] = calibrateSensors(shs_sample, mag_calib_sample, acc_calib_sample, gyro_calib_sample,magnetic)

disp('Calibrating sensor data')

% Sensor's attitude from our iPhone app and from Android app are defined in magnetic north frame 
qMagneticToTrue = dcm2quat(rotz(magnetic.declination));
% shs_sample.device_computed.attitude{:,1:4} = ...
%     quatmultiply(qMagneticToTrue, shs_sample.device_computed.attitude{:,1:4});


% Magnetometer Calibration
[mag_D, mag_bias]=magCalib(mag_calib_sample.raw_imu.magnetometer{:,1:3});
calib_mag = inv(mag_D)*(shs_sample.raw_imu.magnetometer{:,1:3}'- mag_bias);
calib_mag_data = timetable(shs_sample.raw_imu.magnetometer.Time);
calib_mag_data.mag_X = calib_mag(1,:)';
calib_mag_data.mag_Y = calib_mag(2,:)';
calib_mag_data.mag_Z = calib_mag(3,:)';

% Accelerometer Calibration
staticValues = findStaticRegions(acc_calib_sample.raw_imu);

[acc_D,acc_bias] = accCalib(staticValues);
calib_acc = inv(acc_D)*(shs_sample.raw_imu.accelerometer{:,1:3}'- acc_bias);
calib_acc_data = timetable(shs_sample.raw_imu.accelerometer.Time);
calib_acc_data.acc_X = calib_acc(1,:)';
calib_acc_data.acc_Y = calib_acc(2,:)';
calib_acc_data.acc_Z = calib_acc(3,:)';

% Gyroscope Calibration
gyr_bias = gyroCalib(gyro_calib_sample.raw_imu);
calib_gyr = shs_sample.raw_imu.gyroscope{:,1:3}-gyr_bias;
calib_gyr_data = timetable(shs_sample.raw_imu.gyroscope.Time);
calib_gyr_data.gyr_X = calib_gyr(:,1);
calib_gyr_data.gyr_Y = calib_gyr(:,2);
calib_gyr_data.gyr_Z = calib_gyr(:,3);

% Align sensors data
acc = calib_acc_data;
gyr = calib_gyr_data;
mag = calib_mag_data;

firstTimestamp = max([acc.Time(1), gyr.Time(1), mag.Time(1)]);
lastTimestamp = min([acc.Time(end), gyr.Time(end), mag.Time(end)]);

dT = seconds(1/100);
timestamp = firstTimestamp:dT:lastTimestamp;

accSampled = retime(acc,timestamp,'linear');
gyrSampled = retime(gyr,timestamp,'linear');
magSampled = retime(mag,timestamp,'linear');
dev_comp_attitude = retime(shs_sample.device_computed.attitude,timestamp,'linear');

end