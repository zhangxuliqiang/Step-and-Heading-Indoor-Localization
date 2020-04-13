clc
close all
clear all

clear user1_armband
path  = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/oxford step counter/validation/';

gt_sd_data_sets = dir (strcat(path,'/*.csv'));
for gt_sd_data = gt_sd_data_sets'
    disp(['calculating: ' gt_sd_data.name])

file.directory = [gt_sd_data.folder '/'] ;
file.name = gt_sd_data.name;

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","timestamp",(1E-9), ...
    ["X","Y","Z","algo_step_detect","truth_step_detect"]);


[Acceleration, step_detection] = stepDetection(target);
end
%%
GroundTruth = timetable(Acceleration.timestamp);
GroundTruth.step_detect = [0; diff(Acceleration.truth_step_detect)];
step_time = GroundTruth(gt_sd_index(:,1),:).Time;
%
% find the number of step taken according to the ground truth
gt_sd_index = find(diff(Acceleration.truth_step_detect));
[gt_sd_height, ~] = size(gt_sd_index);

% find the number of steps detected by matlab algorithm
matlab_algo_sd_index = find(not(isnan(step_detection.acc4_builtin_max)));
[matlab_algo_sd_height, ~] = size(matlab_algo_sd_index);

% find the number of steps detected by the android algorithm
android_algo_sd_index = find(diff(Acceleration.algo_step_detect));
[android_algo_sd_height, ~] = size(android_algo_sd_index);

gt_sd_height
matlab_algo_sd_height
android_algo_sd_height
%%
clear figure(2)
figure(2)
hax=axes;

hold on
plot(step_detection.Time,step_detection.acc1_conv_gauss)
scatter(step_detection.Time,step_detection.acc4_builtin_max)

% for i = 1:height
%     step_time = TruthData(step_detect_time(i,1),:).timestamp;
%     line([step_time step_time],get(hax,'YLim'),'Color',[1 0 0])
% end

% for i = 1:length(slidding_window)
%     line([slidding_window(i) slidding_window(i)],get(hax,'YLim'),'Color',[0 1 0])
% end
% hold off