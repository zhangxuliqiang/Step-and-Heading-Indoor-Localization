function staticValues = findStaticRegions(rawIMU)

	accelerometer = rawIMU.accelerometer{:,:};
	
	% Retrieve static values
	staticRanges = findStaticPositionsAccelerometer(rawIMU.accelerometer, 1); % static positions for 1 sec
	staticValues = [];
	for i = 1 : length(staticRanges)
		values = accelerometer(staticRanges(i, 1):staticRanges(i, 2), 1:3);
		staticValues(end+1, :) = mean(values);
	end

end