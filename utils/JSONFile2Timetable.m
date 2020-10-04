function [properties,raw_data] = JSONFile2Timetable(dataset)

file = dataset.file;
dataSetProp = dataset.dataSetProp;
file_path = strcat(file.directory, file.name);

target = fileread(file_path);

sample_data = jsondecode(target);
%%
delta_t = dataset.dataSetProp.time_units;

elapsed_time  = transpose(0: delta_t: (length(sample_data.linear_acceleration.x)-1).*delta_t);
data_array = [elapsed_time, ...
        sample_data.linear_acceleration.x,...
        sample_data.linear_acceleration.y,...
        sample_data.linear_acceleration.z,...
        sample_data.orientation.x,...
        sample_data.orientation.y ...
        sample_data.orientation.z];
    
VariableNames = [ dataSetProp.time_var_name, dataSetProp.column_names];
 
raw_data = array2table(data_array,'VariableNames', VariableNames);
raw_data.Time = seconds(raw_data.Time);
raw_data = table2timetable(raw_data);
raw_data = unique(raw_data);

properties = sample_data;
fields = {'linear_acceleration','orientation'};
properties = rmfield(properties,fields);

end