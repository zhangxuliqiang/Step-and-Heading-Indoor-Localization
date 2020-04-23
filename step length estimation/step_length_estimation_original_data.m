clc
close all
clear variables

male.k1 = 0.415;
male.k = 0.3139;

test_height = 1.78;

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
[sl.steps, sl.Acceleration, sl.sd_components] = stepDetection(target, false);

sl.sl_components.period = [0; seconds(diff(sl.steps.data.Time))];
sl.sl_components.period(sl.sl_components.period == 0) = nan;
sl.sl_components.freq = 1./sl.sl_components.period;
sl.mean_freq = mean(sl.sl_components.freq,'omitnan');

sl.step_length = test_height.*male.k.*sqrt(sl.sl_components.freq);
sl.step_length(1) = test_height.*male.k1;
sl.distance_travelled = sum(sl.step_length);

sl_og_data_comparisons = [sl_og_data_comparisons; sl];

end

%%

male.k1 = 0.415;
male.k = 0.3139;

test_height = 1.78;

for comparison = sl_og_data_comparisons'

step_length = test_height.*male.k.*sqrt(comparison.sl_components.freq);
step_length(1) = test_height.*male.k1;
end