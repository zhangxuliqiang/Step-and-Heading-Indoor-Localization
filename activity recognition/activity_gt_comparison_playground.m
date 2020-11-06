close all
clear variables
clc
% Setup the Import Options and import the watch data data

file_path =  '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/activity_recognition/';

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

all_activity = [activity1.sections;activity2.sections;activity3.sections;activity4.sections];
all_activity = sortrows(all_activity);

%% plotting
% plot sections on activity ground truth data

activity  =  activity4;

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
    hold on
    for ii = 1: length(all_activity)
        plot([all_activity(ii,1), all_activity(ii,1)],ylim, 'green')
        plot([all_activity(ii,2), all_activity(ii,2)],ylim, 'red')
        
    end
    hold off
end

title(t,'smart watch gyro and acc with activity regions')
xlabel(t,'time ')
xticklabels(ax(1:end),{})
t.TileSpacing = 'compact';

%% generating features 
clc
window.length = 0.3;
window.step_length = 1; 

feature_watch_data = generateFeatures(all_watch_data, window);

% Converting timetable to table for use by classifier trainer

mw_feature_watch_data = timetable2table(feature_watch_data);

ml_all_watch_data = timetable2table(all_watch_data);

[trainedClassifier, validationAccuracy] = trainClassifier(mw_feature_watch_data);
%%
ar_set.file_path = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/activity_recognition/';
ar_set.watch_file_name ='gt_wessel_iwatch.csv';
ar_set.gt_file_name = 'bart_right_hand_tante_marie.mat';

[specific_watch_data, marie_activity] = generateGroundTruth(ar_set);

specific_feature_watch_data = generateFeatures(specific_watch_data, window);

yfit = trainedClassifier.predictFcn(timetable2table(specific_feature_watch_data));


figure
hold on
plot(specific_feature_watch_data.Time,yfit)
plot(specific_watch_data.Time - specific_watch_data(1,:).Time , specific_watch_data.door_open)
hold off

%% Testing SHS data

clc
pf2.sample_name = 'lopen1.2';
pf2.shs = shs_estimation(pf2.sample_name);
pf2.gt = importVideoGroundTruth([pf2.sample_name '_gt_from_video.csv']);

file_path  = 'datasets/marie testing/lopen1.2/Wessel’s Apple Watch 201020 16_50_13.csv';
watch_data = AccWatchFile2Timetable(file_path);

record = importStartTime("/home/default/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/marie testing/lopen1.2/lopen1_2/record.properties", [13, 16]);

time_difference = record.StartTime - watch_data(1,:).Time;

door_handle_use = ReferenceFile2Timetable('datasets/marie testing/lopen1.2/lopen1_2/references.txt');

door_handle_use.elapsed = door_handle_use.elapsed + record.StartTime;

%%
yfit = trainedClassifier.predictFcn(timetable2table(feature_new_data));

%%
figure
hold on
% plot(feature_new_data.Time,yfit,'x')
plot(door_handle_use.elapsed ,door_handle_use.door_detection,'x')
hold off
ylim([0,1.5])
ylabel('acitivity recognition')
xlabel('Time')
legend('AR','ground truth')
title('activity recognition (1 = activity recognized)')
