
function aData = loadiphoneDataset(directory)

    aData = AcquisitionData;
    aData.provider = 'iphone';

    aData.data_directory = directory;


%     if exist(directory, 'file') ~= 7
%         error([directory ' does not exists'])
%     end

    iphone = iOSFile2Timetable(directory);

    % Raw data
    raw_imu = aData.raw_imu;
    gyro_timetable = timetable(iphone.Time);
    gyro_timetable.X = iphone.gyroRotationX;
    gyro_timetable.Y = iphone.gyroRotationY;
    gyro_timetable.Z = iphone.gyroRotationZ;

    raw_imu.gyroscope = gyro_timetable;

    acc_timetable = timetable(iphone.Time);
    acc_timetable.X = iphone.accelerometerAccelerationX.*9.81;
    acc_timetable.Y = iphone.accelerometerAccelerationY.*9.81;
    acc_timetable.Z = iphone.accelerometerAccelerationZ.*9.81;

    raw_imu.accelerometer = acc_timetable;


    mag_timetable = timetable(iphone.Time);
    mag_timetable.X = iphone.magnetometerX;
    mag_timetable.Y = iphone.magnetometerY;
    mag_timetable.Z = iphone.magnetometerZ;

    raw_imu.magnetometer = mag_timetable;
    
    device_computed = aData.device_computed;
    
    att_timetable = timetable(iphone.Time);
    att_timetable.Yaw = iphone.motionYaw;
    att_timetable.Roll = iphone.motionRoll;
    att_timetable.Pitch = iphone.motionPitch;
	device_computed.attitude = att_timetable;

end




