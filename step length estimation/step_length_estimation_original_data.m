clc
% close all
% clear variables
clear path

male_const.k1 = 0.415;
male_const.k = 0.308;

test_height = 1.80;

sl_og_data_comparisons = [];

path.path = '/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code and Datasets/SHS Code/datasets/step length estimation/';
path.time_unit = 1E-3;

sl_datasets = dir(strcat(path.path,'/*.csv'));
for sl_dataset = sl_datasets'

file.directory = [sl_dataset.folder '/'] ;
file.name = sl_dataset.name;

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","Time",(path.time_unit), ...
    ["X","Y","Z"]);
sl.name = file.name;
[sl.steps, sl.Acceleration, sl.sd_components] = stepDetection(target,'file', false);

sl.sl_components.period = [0; seconds(diff(sl.steps.data.Time))];
sl.sl_components.period(sl.sl_components.period == 0) = nan;
sl.sl_components.freq = 1./sl.sl_components.period;
sl.mean_freq = mean(sl.sl_components.freq,'omitnan');

sl.step_length = test_height.*male_const.k.*sqrt(sl.sl_components.freq);
sl.step_length(1) = test_height.*male_const.k;
sl.distance_travelled = sum(sl.step_length);

sl_og_data_comparisons = [sl_og_data_comparisons; sl];

end

%% absolute distance travelled

bar_comp_data = [];
bar_names = {};
n = 0;
figure()
for comparison = sl_og_data_comparisons'
   n = n + 1;
   % formatting name for nice plotting by replacing 
   name_format = strrep(comparison.name,'_',' ');
   
   %cut the string at the char 5 so that date info is not seen in plot
   name = strsplit(name_format,'H');
   
   %only use the name
   bar_names(n) = {name{1}(1:end-1)}; 
   bar_comp_data = [ bar_comp_data; comparison.distance_travelled];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
ylabel('distance travelled (m)')
title('distance travelled based on steps counted and frequency step length') 

%% absolute distance travelled

bar_comp_data = [];
bar_names = {};
n = 0;
figure()
for comparison = sl_og_data_comparisons'
   n = n + 1;
   % formatting name for nice plotting by replacing 
   name_format = strrep(comparison.name,'_',' ');
   
   %cut the string at the char 5 so that date info is not seen in plot
   name = strsplit(name_format,'H');
   
   %only use the name
   bar_names(n) = {name{1}(1:end-1)}; 
   bar_comp_data = [ bar_comp_data; ((comparison.distance_travelled - 60.5) / 60.5)*100 ];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
ylabel('percentage error (%)')
title('percentage error based on steps counted and frequency step length') 