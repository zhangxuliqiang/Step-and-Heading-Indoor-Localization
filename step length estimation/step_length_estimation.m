close all
clear variables
clc

path = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/Online Code/SLERepository-master/SLEDataset/person02/';
% find all files in directory and sub directories
sl_datasets = dir(strcat(path,'**/*.json'));

sl_comparisons = [];

for sl_dataset = sl_datasets'
    clear steps
    is_testpath1 = strfind(sl_dataset.folder,'test_path1');
    
    if ~isempty(is_testpath1)
        file.directory = [sl_dataset.folder '/'] ;
        file.name = sl_dataset.name;
        
        target.file = file;
        target.dataSetProp = DataSetProp("Accelerometer","Time",(1E-2), ...
            ["X","Y","Z"]);
        
        [sd.steps, sd.Acceleration, sd.sd_components] = stepDetection(target, false);
        
        if ~isempty(sd.steps.data)
            file_path = strcat(file.directory, file.name);
            read_file = fileread(file_path);          
            sd.name = file.name;
            sd.dataset_info = jsondecode(read_file);
            sd.sl_components.period = [0; seconds(diff(sd.steps.data.Time))];
            sd.sl_components.period(sd.sl_components.period == 0) = nan;
            sd.sl_components.freq = 1./sd.sl_components.period;
            sd.mean_freq = mean(sd.sl_components.freq,'omitnan');
            sl_comparisons = [sl_comparisons, sd];
        else
            disp('no steps detected')
        end        
        
    end
end

%% 
close all

slow_walk_names = strfind({sl_comparisons.name},'normal')';
nonzeroCells = ~cellfun('isempty',slow_walk_names);
slow_walk_index = find(nonzeroCells);
slow_walk_data = sl_comparisons(slow_walk_index);

for i = 1:length(slow_walk_data)
    figure(i)
    stackedplot(slow_walk_data(i).sd_components)
end

spreadfigures

%% plot
figure(i+1)
plot([sl_comparisons.nr_steps])

%%
figure(2)
hold on
plot(sd.sd_components.Time, sd.sd_components.acc1_conv_gauss)
scatter(sd.sd_components.Time,sd.sd_components.acc4_builtin_max)
hold off
%%
figure()
plot(sd.Time,sd.frequency)
title('step frequency')