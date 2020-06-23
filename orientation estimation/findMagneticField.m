function magnetic = findMagneticField(location, date)

	% Magnetic field context
	[magnetic_vector, ~, magnetic_declination, magnetic_inclination, magnetic_magnitude] = ...
		wrldmagm(location.altitude, location.latitude, location.longitude, ...
			decyear(date.year, date.month, date.day));

	% Transform nanoTesla to microTesla
	magnetic.magnitude = magnetic_magnitude / 1000;
	magnetic_vector = magnetic_vector / 1000;
    
    % the reference frame of the World Magnetic Model is NED, while ENU is
    % used for orientation estimation. Conversion is required !
    
    rotmatNEDtoENU = roty(180)*rotz(90);   
    
    magnetic.vector = rotmatNEDtoENU * magnetic_vector;
    magnetic.declination = -magnetic_declination;
    magnetic.inclination = magnetic_inclination;

end