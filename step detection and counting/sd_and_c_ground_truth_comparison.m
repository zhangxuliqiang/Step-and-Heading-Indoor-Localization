clc
close all
clear all

gt2algo_comparisons = [];

path.path  = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/oxford step counter/validation/';
path.time_unit = 1E-9;

gt_sd_datasets = dir(strcat(path.path,'/*.csv'));
%
for gt_sd_dataset = gt_sd_datasets'

file.directory = [gt_sd_dataset.folder '/'] ;
file.name = gt_sd_dataset.name;

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","Time",(path.time_unit), ...
    ["X","Y","Z","algo_step_detect","truth_step_detect"]);

sd.name = file.name;

[sd.matlab_algo_steps, sd.Acceleration, sd.sd_components] = stepDetection(target, false);

sd.sd_comparison = createTimeSeriesCompare(sd.Acceleration, sd.matlab_algo_steps.data);

% find the step time series of android and ground truth 
sd.android_algo_steps = findSteps(sd.sd_comparison.sd_android_algo_points, sd.Acceleration);
sd.ground_truth_steps = findSteps(sd.sd_comparison.sd_ground_truth_points, sd.Acceleration);

parfor result_index = 1 : 25
    debugDisp([ gt_sd_dataset.name "- delta_t: " result_index], true)
    delta_t = result_index * 0.02; 
    pseudo_confusion(result_index) = TpFpFnCalc(sd.matlab_algo_steps.data, sd.ground_truth_steps.data,delta_t);
end

sd.pseudo_confusion = pseudo_confusion;

gt2algo_comparisons = [gt2algo_comparisons; sd];

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