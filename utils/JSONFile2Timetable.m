function [Data, elapsed] = JSONFile2Timetable(dataset)

file = dataset.file;
dataSetProp = dataset.dataSetProp;
file_path = strcat(file.directory, file.name);

target = fileread(file_path);

step_length_data = jsondecode(target);
%%
delta_t = dataset.dataSetProp.time_units;

elapsed_time  = transpose(0: delta_t: (length(step_length_data.linear_acceleration.x)-1).*delta_t);
data_array = [elapsed_time, ...
        step_length_data.linear_acceleration.x,...
        step_length_data.linear_acceleration.y ...
        step_length_data.linear_acceleration.z];
    
VariableNames = [ dataSetProp.time_var_name, dataSetProp.column_names];
 
Data = array2table(data_array,'VariableNames', VariableNames);
Data.Time = seconds(Data.Time);
Data = table2timetable(Data);
Data = unique(Data);

end