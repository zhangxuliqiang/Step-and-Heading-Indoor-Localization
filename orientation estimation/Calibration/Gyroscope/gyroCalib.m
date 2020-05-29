function bias = gyroCalib(rawIMU)

	gyroscope = rawIMU.gyroscope{:,:};

	staticRanges = findStaticPositionsGyroscope(rawIMU.gyroscope, 2); % static positions for 2 sec
	staticValues = [];

	for i = 1 : size(staticRanges,1)
		staticValues = [ staticValues ; gyroscope(staticRanges(i, 1):staticRanges(i, 2), 1:3)];
	end

	bias = mean(staticValues);

end