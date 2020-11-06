close all
clear variables
clc

%% Testing SHS data

clc
pf2.sample_name = 'lopen1.2';
pf2.shs = shs_estimation(pf2.sample_name);
pf2.gt = importVideoGroundTruth([pf2.sample_name '_gt_from_video.csv']);

file_path  = 'datasets/marie testing/lopen1.2/Wessel’s Apple Watch 201020 16_50_13.csv';
new_data = AccWatchFile2Timetable(file_path);

feature_new_data = generateFeatures(new_data, window);

record = importStartTime("/home/default/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/marie testing/lopen1.2/lopen1_2/record.properties", [13, 16]);

time_difference = record.StartTime - new_data(1,:).Time;

door_handle_use = ReferenceFile2Timetable('datasets/marie testing/lopen1.2/lopen1_2/references.txt');

door_handle_use.elapsed = door_handle_use.elapsed + record.StartTime;

