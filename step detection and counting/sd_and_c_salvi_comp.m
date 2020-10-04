% SCRIPT USE
% the following script use the datasets made available by salvi et al to
% compare groud truth steps, salvi et al algorithm and the variation made
% in matlab

% VARIABLE LEGEND 
% sd = step detection
% gt = ground truth

clc
close all
clear variables

% initialize comparison struct that will be populated later
gt2algo_comparisons = [];

% define path in which the datasets can be found 
path.path  = ['/home/' getenv('USER') ...
    '/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/oxford step counter/validation/'];

% Salvi et al. dataset timestamp is in nano seconds
path.time_unit = 1E-9;

% find all files in the directory that have csv extension
gt_sd_datasets = dir(strcat(path.path,'/*.csv'));

for gt_sd_dataset = gt_sd_datasets'
    
    % populate import struct
    file.directory = [gt_sd_dataset.folder '/'] ;
    file.name = gt_sd_dataset.name;
    
    target.file = file;
    target.dataSetProp = DataSetProp("Accelerometer","Time",(path.time_unit), ...
        ["X","Y","Z","algo_step_detect","truth_step_detect"]);
    
    sd.name = file.name;
    
    % perform step detection for the specfied dataset
    [sd.matlab_algo_steps, sd.raw_data, sd.sd_components] = ...
        stepDetection(target, 'file', false);
    
    % create comparison struct between gt, salvi algo, and matlab algo
    sd_comparison.name = file.name;
    
    sd_comparison.raw_data = sd.raw_data;
    
    % add step detection flags to data timetable for later analysis
    sd_comparison.ts_comparison = ...
        createTimeSeriesCompare(sd.raw_data, sd.matlab_algo_steps.data);
    
    % find the step time series and number of steps of android and ground 
    % truth
    
    sd_comparison.salvi_algo_steps = ...
        findSteps(sd_comparison.ts_comparison.sd_salvi_algo_points, ...
                  sd.raw_data);
    
    sd_comparison.ground_truth_steps = ...
        findSteps(sd_comparison.ts_comparison.sd_ground_truth_points, ...
                  sd.raw_data);
    
    sd_comparison.matlab_algo_steps = sd.matlab_algo_steps;
    
    % determine true positives, false positives and false negatives within
    % a time range, which is itterated over to find the range in which the
    % the best result can be found
    
%     for result_index = 1 : 25
%         result_index
%         debugDisp([ gt_sd_dataset.name "- delta_t: " num2str(result_index)], true)
%         delta_t = result_index * 0.02;
%         
%         matlab_pseudo_confusion(result_index) = ... 
%             TpFpFnCalc(sd_comparison.matlab_algo_steps.data, ...
%                        sd_comparison.ground_truth_steps.data,delta_t);
%                    
%         android_pseudo_confusion(result_index) = ...
%             TpFpFnCalc(sd_comparison.salvi_algo_steps.data, ...
%                        sd_comparison.ground_truth_steps.data,delta_t);
%     end
%     
%     sd_comparison.matlab_pseudo_confusion = matlab_pseudo_confusion;
%     sd_comparison.android_pseudo_confusion = android_pseudo_confusion;
    
    gt2algo_comparisons = [gt2algo_comparisons; sd_comparison];
    
end
