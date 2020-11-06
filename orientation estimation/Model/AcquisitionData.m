
classdef AcquisitionData < handle

	properties(Access = public)

		provider = 'Unknown'
		data_directory

		%  === Raw ===
		raw_imu
        door_handle_use

		%  === Device computed ===
		device_computed

		% == Used by code (default is from device computed) ==

	end
	

	methods(Access = public)

		function obj = AcquisitionData(obj)
			obj.raw_imu = IMUData;
			obj.device_computed = DeviceComputedData;
            obj.door_handle_use = [];
        end

	end
end
