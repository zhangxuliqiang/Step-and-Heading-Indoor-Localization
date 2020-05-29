
function aData = loadAndroidDataset(directory)

	aData = AcquisitionData;
	aData.provider = 'Android';

	aData.data_directory = directory;


	if exist(directory, 'file') ~= 7 
		error([directory ' does not exists'])
	end


	% Raw data
	raw_imu = aData.raw_imu;
    gyr_var_names = ["gyr_X","gyr_Y","gyr_Z"];
	raw_imu.gyroscope = loadIfExists(directory, 'gyroscope.txt', gyr_var_names);
    
    acc_var_names = ["acc_X","acc_Y","acc_Z"];
	raw_imu.accelerometer = loadIfExists(directory, 'accelerometer.txt', acc_var_names); 
    
    mag_var_names = ["mag_X","mag_Y","mag_Z"];
	raw_imu.magnetometer = loadIfExists(directory, 'magnetometer.txt',mag_var_names);


	% Device computed data
	device_computed = aData.device_computed;
    gyr_cal_var_names = ["gyr_cal_X","gyr_cal_Y","gyr_cal_Z"];
	device_computed.gyroscope = loadIfExists(directory, 'gyroscope-calibrated.txt',gyr_cal_var_names);
	
    mag_cal_var_names = ["mag_cal_X","mag_cal_Y","mag_cal_Z"];
    device_computed.magnetometer = loadIfExists(directory, 'magnetometer-calibrated.txt',mag_cal_var_names);
	
    att_var_names = ["att_q1","att_q2","att_q3","att_q4"];
    device_computed.attitude = loadIfExists(directory, 'rotation-vector.txt',att_var_names);
	
	% Quaternion gave by Android is [q2 q3 q4 q1]
	% http://developer.android.com/reference/android/hardware/SensorEvent.html#values
	if ~isempty(device_computed.attitude)
		device_computed.attitude{:, 1:4} = device_computed.attitude{:, [4 1 2 3]};
		device_computed.attitude{:, 1:4} = removeQuaternionsJumps(device_computed.attitude{:, 1:4});
    end
	
end

function outputMatrix = loadIfExists(directory, file_name, variable_names)

	file_path = [ directory file_name];
    
    file.directory = directory;
    file.name = file_name;
    file.time_unit = 1;

    target.file = file;
    target.dataSetProp = DataSetProp(file_name,"Time",(file.time_unit), ...
        variable_names);
        
	if exist(file_path, 'file') == 2
		outputMatrix = SSVFile2Timetable(target);
	else
		outputMatrix = [];
%         disp([file_path ' is not available'])
	end

end


