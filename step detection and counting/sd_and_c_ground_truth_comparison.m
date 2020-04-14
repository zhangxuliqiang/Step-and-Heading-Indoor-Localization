clc
close all
clear all

gt2algo_comparison = [];

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

% find the number of step taken according to the ground truth
gt_sd_index = find(diff(Acceleration.truth_step_detect));
[gt_sd_height, ~] = size(gt_sd_index);

% find the number of steps detected by matlab algorithm
matlab_algo_sd_index = find(not(isnan(step_detection.acc4_builtin_max)));
[matlab_algo_sd_height, ~] = size(matlab_algo_sd_index);

% find the number of steps detected by the android algorithm
android_algo_sd_index = find(diff(Acceleration.algo_step_detect));
[android_algo_sd_height, ~] = size(android_algo_sd_index);

output.name = file.name;
output.gt_sd_height = gt_sd_height;
output.matlab_algo_sd_height = matlab_algo_sd_height;
output.android_algo_sd_height = android_algo_sd_height;

gt2algo_comparison = [gt2algo_comparison; output];

end
%% Absolute number of steps detected

bar_comp_data = [];
bar_names = {};
n = 0;
figure(1)
for comparison = gt2algo_comparison'
   n = n + 1;
   % formatting name for nice plotting
   name = strsplit(strrep(comparison.name,'_',' '),'5');
   bar_names(n) = {name{1}(1:end-2)}; 
   bar_comp_data = [ bar_comp_data; comparison.gt_sd_height, comparison.matlab_algo_sd_height, ...
                comparison.android_algo_sd_height];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
title('step counting comparison with lower standard deviation threshold') 
legend('ground truth','matlab algorithm','android algorithm')

%% Percentual accuracy

bar_comp_data = [];
bar_names = {};
n = 0;
figure(2)
for comparison = gt2algo_comparison'
   n = n + 1;
   % formatting name for nice plotting
   name = strsplit(strrep(comparison.name,'_',' '),'5');
   bar_names(n) = {name{1}(1:end-2)}; 
   
   %grouping dataset together
   bar_comp_data = [ bar_comp_data;  ... 
                (comparison.matlab_algo_sd_height - comparison.gt_sd_height)./comparison.gt_sd_height *100, ...
                (comparison.android_algo_sd_height -comparison.gt_sd_height)./comparison.gt_sd_height * 100];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
title("step counting error with lower standard deviation threshold") 
legend('matlab algorithm','android algorithm')

%%

% GroundTruth = timetable(Acceleration.timestamp);
% GroundTruth.step_detect = [0; diff(Acceleration.truth_step_detect)];
% step_time = GroundTruth(gt_sd_index(:,1),:).Time;
% %
% 
% clear figure(2)
% figure(2)
% hax=axes;
% 
% hold on
% plot(step_detection.Time,step_detection.acc1_conv_gauss)
% scatter(step_detection.Time,step_detection.acc4_builtin_max)

% for i = 1:height
%     step_time = TruthData(step_detect_time(i,1),:).timestamp;
%     line([step_time step_time],get(hax,'YLim'),'Color',[1 0 0])
% end

% for i = 1:length(slidding_window)
%     line([slidding_window(i) slidding_window(i)],get(hax,'YLim'),'Color',[0 1 0])
% end
% hold off