function feature_watch_data = generateFeatures(target,window)

target.Time = duration(target.Time - target(1,:).Time); 

target.Time.Format = 'hh:mm:ss.SSS';

[~,ia,ic] = unique(target.Time);

target  = target(ia,:);

% calculate moving standard deviation and moving mean with a trailing
% sliding window
std_watch_data = movstd(target{:,:},[seconds(window.length),seconds(0)], ...
                        'SamplePoints',target.Time);

mean_watch_data = movmean(target{:,:},[seconds(window.length),seconds(0)], ...
                          'SamplePoints',target.Time);

% Add standard deviation and mean in timetable with same time as watch
% table
feature_watch_data = timetable(target.Time, ...
                               std_watch_data(:,1:end-1),mean_watch_data(:,1:end-1) );

% Adding appropriate name to table columns
feature_watch_data = splitvars(feature_watch_data);
std_name = append( 'std_' , target.Properties.VariableNames(:,1:end-1));
mean_name = append( 'mean_' , target.Properties.VariableNames(:,1:end-1));
feature_watch_data.Properties.VariableNames =[std_name, mean_name] ;

feature_watch_data.door_open = target.door_open;

% sampling features with window.step_length interval
time_range  = seconds(0):seconds(window.step_length):target.Time(end);

% time is not exact so tolerance is used
time_range_tol = withtol(time_range,seconds(0.01));
feature_watch_data = feature_watch_data(time_range_tol,:);

end