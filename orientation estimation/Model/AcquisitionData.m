
classdef AcquisitionData < handle

	properties(Access = public)

		provider = 'Unknown'
		data_directory

		%  === Raw ===
		raw_imu

		%  === Device computed ===
		device_computed

		% == Used by code (default is from device computed) ==

	end
	

	methods(Access = public)

		function obj = AcquisitionData(obj)
			obj.raw_imu = IMUData;
			obj.device_computed = DeviceComputedData;
        end

	end
end
