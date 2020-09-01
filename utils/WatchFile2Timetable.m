function data  = WatchFile2Timetable(file_path)

opts = delimitedTextImportOptions("NumVariables", 23);

% Specify range and delimiter
opts.DataLines = [2, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Time", "accelerometerTimestamp_sinceReboots", "accelerometerAccelerationXG", "accelerometerAccelerationYG", "accelerometerAccelerationZG", "motionTimestamp_sinceReboots", "motionYawrad", "motionRollrad", "motionPitchrad", "motionRotationRateXrads", "motionRotationRateYrads", "motionRotationRateZrads", "motionUserAccelerationXG", "motionUserAccelerationYG", "motionUserAccelerationZG", "Var16", "motionQuaternionXR", "motionQuaternionYR", "motionQuaternionZR", "motionQuaternionWR", "motionGravityXG", "motionGravityYG", "motionGravityZG"];
opts.SelectedVariableNames = ["Time", "accelerometerTimestamp_sinceReboots", "accelerometerAccelerationXG", "accelerometerAccelerationYG", "accelerometerAccelerationZG", "motionTimestamp_sinceReboots", "motionYawrad", "motionRollrad", "motionPitchrad", "motionRotationRateXrads", "motionRotationRateYrads", "motionRotationRateZrads", "motionUserAccelerationXG", "motionUserAccelerationYG", "motionUserAccelerationZG", "motionQuaternionXR", "motionQuaternionYR", "motionQuaternionZR", "motionQuaternionWR", "motionGravityXG", "motionGravityYG", "motionGravityZG"];
opts.VariableTypes = ["string", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "string", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts.ConsecutiveDelimitersRule = "join";

% Specify variable properties
opts = setvaropts(opts, ["Time", "Var16"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Time", "Var16"], "EmptyFieldRule", "auto");

% Import the data
data = readtable(file_path, opts);

data.Time = datetime(data.Time,'InputFormat','yyyy-MM-dd HH:mm:ss.SSS X','TimeZone','local') ;

data = table2timetable(data);

data.door_open = zeros(height(data),1);

end