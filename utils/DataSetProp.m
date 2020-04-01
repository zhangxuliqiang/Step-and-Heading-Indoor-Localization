classdef DataSetProp  < handle
    %DATASET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties ( Access = public )
        measurement_var 
        time_var_name
        time_units
        measurement_x
        measurement_y
        measurement_z
    end
    
    methods
        function obj = DataSetProp(measurement_var, time_var_name, time_units, ...
                measurement_x, measurement_y, measurement_z)
            
        obj.measurement_var = measurement_var;
        obj.time_var_name   = time_var_name;
        obj.time_units      = time_units;
        obj.measurement_x   = measurement_x;
        obj.measurement_y   = measurement_y;
        obj.measurement_z   = measurement_z;

        end
    end
end

