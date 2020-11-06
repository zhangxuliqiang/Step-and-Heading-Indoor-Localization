function data  = AccWatchFile2Timetable(file_path)

opts = delimitedTextImportOptions("NumVariables", 5);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Time", "accelerometerTimestamp_sinceReboots", "accelerometerAccelerationXG", "accelerometerAccelerationYG", "accelerometerAccelerationZG"];
opts.VariableTypes = ["string", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, "Time", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Time", "EmptyFieldRule", "auto");

% Import the data
data = readtable(file_path, opts);

data.Time = datetime(data.Time,'InputFormat','yyyy-MM-dd HH:mm:ss.SSS X','TimeZone','local') ;

data = table2timetable(data);

data.door_open = zeros(height(data),1);

end