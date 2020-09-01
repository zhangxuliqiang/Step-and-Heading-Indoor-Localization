close all
clear variables
clc
% Setup the Import Options and import the watch data data

file_path = "/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/activity_recognition/watchdata4_withsystime.csv";

watch_data = WatchFile2Timetable(file_path);

% load ground truth timeset in which a door handle was used

load('/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/activity_recognition/handleuse4_withsystime.mat')
activity_time = datetime(data(:,1),'ConvertFrom','datenum');
activity_time.TimeZone = 'local';
activity_time.Format = 'yyyy-MM-dd HH:mm:ss.SSS';

% find sections in which activity occurs
activity_sections = [];
start_section = activity_time(1);

for i  = 2: length(activity_time)
    time_diff  = seconds(activity_time(i)-activity_time(i-1));
    
    if time_diff> 1
        
        end_section = activity_time(i-1);
        activity_sections = [activity_sections; start_section, end_section];
        
        start_section  = activity_time(i);
    end    
    
end

% remove sections that have the same start and end time 
activity_sections(activity_sections(:,1) == activity_sections(:,2),: ) = [];



%% plotting
% plot sections on activity ground truth data
figure()
plot(activity_time)
grid on;
hold on;
for i = 1: length(activity_sections)
    plot(xlim, [activity_sections(i,1), activity_sections(i,1)], 'green')
    plot(xlim, [activity_sections(i,2), activity_sections(i,2)], 'red')
end

% plot sections on watch data

selected_data = [2:4, 9:11];

t = tiledlayout(length(selected_data),1);

for index  = 1: length(selected_data)
    i = selected_data(index);
    ax(index) = nexttile;
    plot(ax(index),watch_data.Time,  watch_data{:,i})
    title( watch_data.Properties.VariableNames(i), 'FontSize',5   )
    hold on
    for ii = 1: length(activity_sections)
        plot([activity_sections(ii,1), activity_sections(ii,1)],ylim, 'green')
        plot([activity_sections(ii,2), activity_sections(ii,2)],ylim, 'red')
        
    end
    hold off
end

title(t,'smart watch gyro and acc with activity regions')
xlabel(t,'time ')
xticklabels(ax(1:end),{})
t.TileSpacing = 'compact';


for i  = 1: length(activity_sections)
   tr_activity = timerange(activity_sections(i,1), activity_sections(i,2));
   
   watch_data(tr_activity,:).door_open  = ones(height( watch_data(tr_activity,:)),1);
end

%% Now that the activity is synchronized change datetime to duration
watch_data.Time = duration(watch_data.Time - watch_data(1,:).Time); 
watch_data.Time.Format = 'mm:ss.SSS';

%% generating features 
window_length = 1;
window_step_length = 1; 

% calculate moving standard deviation and moving mean with a trailing
% sliding window
std_watch_data = movstd(watch_data{:,:},[seconds(window_length),seconds(0)], ...
                        'SamplePoints',watch_data.Time);

mean_watch_data = movmean(watch_data{:,:},[seconds(window_length),seconds(0)], ...
                          'SamplePoints',watch_data.Time);

% Add standard deviation and mean in timetable with same time as watch
% table
feature_watch_data = timetable(watch_data.Time, ...
                               std_watch_data(:,1:end-1),mean_watch_data );

% Adding appropriate name to table columns
feature_watch_data = splitvars(feature_watch_data);
std_name = append( 'std_' , watch_data.Properties.VariableNames(:,1:end-1));
mean_name = append( 'mean_' , watch_data.Properties.VariableNames);
feature_watch_data.Properties.VariableNames =[std_name, mean_name] ;

% change last column (which is mean_door_open) to door_open and equal to 
% watch_data door_open
feature_watch_data.Properties.VariableNames(end) = {'door_open'};
feature_watch_data.door_open = watch_data.door_open;

% sampling features with window_step_length interval
time_range  = seconds(0):seconds(window_step_length):watch_data.Time(end);

% time is not exact so tolerance is used
time_range_tol = withtol(time_range,seconds(0.01));
mw_feature_watch_data = feature_watch_data(time_range_tol,:);

%% Converting timetable to table for use by classifier trainer

mw_feature_watch_data = timetable2table(mw_feature_watch_data);

ml_watch_data = timetable2table(watch_data);


[trainedClassifier, validationAccuracy] = trainClassifier(ml_watch_data);

yfit = trainedClassifier.predictFcn(ml_watch_data);

%%
% using a handleuse2 dataset 
clc
file_path2  = "/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/activity_recognition/watchdata2.csv";

new_data = WatchFile2Timetable(file_path2);

new_data.Time = duration(new_data.Time - new_data.Time(1));

new_data.Time.Format = 'mm:ss.SSS';

ml_new_data = timetable2table(new_data);

yfit = trainedClassifier.predictFcn(ml_new_data);

data_handleuse2_tt = timetable(seconds(data_handleuse2(:,1)));

data_handleuse2_tt.door_open = data_handleuse2(:,2);



figure()
plot(data_handleuse2_tt.Time, data_handleuse2_tt.door_open>0,'*')
hold on
plot(new_data.Time,yfit)
hold off

