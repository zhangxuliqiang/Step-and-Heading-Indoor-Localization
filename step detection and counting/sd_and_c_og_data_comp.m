%% sd_and_c_salvi_comp

% SCRIPT USE
% the following script use the datasets made available by salvi et al to
% compare groud truth steps, salvi et al algorithm and the variation made
% in matlab

% VARIABLE LEGEND 
% sd = step detection
% gt = ground truth

%%
clc
close all
clear variables

% initialize comparison struct that will be populated later

sd_og_data_comparisons = [];

% define path in which the datasets can be found 
path.path = ['/home/' getenv('USER') ...
    '/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/step counting/20 april/'];

% HIMU app is in milliseconds 
path.time_unit = 1E-3;

% find all files in the directory that have csv extension
gt_sd_datasets = dir(strcat(path.path,'/*.csv'));
for gt_sd_dataset = gt_sd_datasets'
    
    % populate import struct    
    file.directory = [gt_sd_dataset.folder '/'] ;
    file.name = gt_sd_dataset.name;
    
    target.file = file;
    target.dataSetProp = DataSetProp("Accelerometer","Time",(path.time_unit), ...
        ["X","Y","Z"]);
    
    sd.name = file.name;
    
    % perform step detection for the specfied dataset
    [sd.steps, sd.Acceleration, sd.sd_components] = stepDetection(target,'file', false);
    
    sd_og_data_comparisons = [sd_og_data_comparisons; sd];

end
