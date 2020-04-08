classdef DataSetProp  < handle
    %DATASET Summary of this class goes here
    %   Detailed explanation goes here
    
    properties ( Access = public )
        measurement_var 
        time_var_name
        time_units
        column_names
    end
    
    methods
        function obj = DataSetProp(measurement_var, time_var_name, time_units, ...
                column_names)
            
        obj.measurement_var = measurement_var;
        obj.time_var_name   = time_var_name;
        obj.time_units      = time_units;
        obj.column_names   = column_names;

        end
    end
end

