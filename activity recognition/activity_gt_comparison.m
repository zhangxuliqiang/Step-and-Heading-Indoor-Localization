close all
clear variables
clc
% Setup the Import Options and import the watch data data

file_path =  '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/activity recognition/';

ar_set1.file_path = file_path;
ar_set1.watch_file_name ='jWatch 200820 22_19_38.csv';
ar_set1.gt_file_name = 'left_hand_bart.mat';

[watch_data1, activity1] = generateGroundTruth(ar_set1);

ar_set2.file_path = file_path;
ar_set2.watch_file_name ='jWatch 200820 22_32_42.csv';
ar_set2.gt_file_name = 'right_hand_bart.mat';

[watch_data2, activity2] = generateGroundTruth(ar_set2);

ar_set3.file_path = file_path;
ar_set3.watch_file_name ='jWatch 200820 22_49_44.csv';
ar_set3.gt_file_name = 'left_hand_julia.mat';

[watch_data3, activity3] = generateGroundTruth(ar_set3);

ar_set3.file_path = file_path;
ar_set3.watch_file_name ='jWatch 200820 22_44_21.csv';
ar_set3.gt_file_name = 'right_hand_julia.mat';

[watch_data4, activity4] = generateGroundTruth(ar_set3);

all_watch_data = [watch_data1;watch_data2;watch_data3;watch_data4];
all_watch_data = sortrows(all_watch_data);
all_watch_data.Time.Format = 'dd-MMM-uuuu HH:mm:ss.SSS';
%% plotting
% plot sections on activity ground truth data
figure()
plot(activity.time)
grid on;
hold on;
for i = 1: length(activity.sections)
    plot(xlim, [activity.sections(i,1), activity.sections(i,1)], 'green')
    plot(xlim, [activity.sections(i,2), activity.sections(i,2)], 'red')
end

%% plot sections on watch data

figure()
selected_data = [2:4, 9:11];

t = tiledlayout(length(selected_data),1);

for index  = 1: length(selected_data)
    i = selected_data(index);
    ax(index) = nexttile;
    plot(ax(index),all_watch_data.Time,  all_watch_data{:,i})
    title( all_watch_data.Properties.VariableNames(i), 'FontSize',5   )
%     hold on
%     for ii = 1: length(activity.sections)
%         plot([activity.sections(ii,1), activity.sections(ii,1)],ylim, 'green')
%         plot([activity.sections(ii,2), activity.sections(ii,2)],ylim, 'red')
%         
%     end
%     hold off
end

title(t,'smart watch gyro and acc with activity regions')
xlabel(t,'time ')
xticklabels(ax(1:end),{})
t.TileSpacing = 'compact';

%% generating features 
clc
window_length = 1;
window_step_length = 1; 

target = all_watch_data;
target.Time = duration(target.Time - target(1,:).Time); 

target.Time.Format = 'hh:mm:ss.SSS';

[~,ia,ic] = unique(target.Time);

target  = target(ia,:);

% calculate moving standard deviation and moving mean with a trailing
% sliding window
std_watch_data = movstd(target{:,:},[seconds(window_length),seconds(0)], ...
                        'SamplePoints',target.Time);

mean_watch_data = movmean(target{:,:},[seconds(window_length),seconds(0)], ...
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

% sampling features with window_step_length interval
time_range  = seconds(0):seconds(window_step_length):target.Time(end);

% time is not exact so tolerance is used
time_range_tol = withtol(time_range,seconds(0.01));
feature_watch_data = feature_watch_data(time_range_tol,:);

%% Converting timetable to table for use by classifier trainer

mw_feature_watch_data = timetable2table(feature_watch_data);

ml_all_watch_data = timetable2table(all_watch_data);


[trainedClassifier, validationAccuracy] = trainClassifier(ml_all_watch_data);

% yfit = trainedClassifier.predictFcn(ml_all_watch_data);

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

