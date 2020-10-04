%% sd_and_c_og_data_comp_plotter_scripts

% SCRIPT USE
% The following script is used to plot different performance measures of the 
% stepDetection function. They are sectioned so that they can be run
% seperately with ctrl=shift

% NOTE
% the script is not standalone and requires the workspace to contain
% the output of sd_and_c_salvi_comp.m

%% Percentual accuracy
% the data generate had the user walk exactly 60 steps. Knowing this, the
% percentage error can be calculated for the different carrying modes.

bar_comp_data = [];
bar_names = {};
n = 0;
figure()
for comparison = sd_og_data_comparisons'
      n = n + 1;
   % formatting name for nice plotting by replacing underscore with space
   name_format = strrep(comparison.name,'_',' ');
   
   %cut the string at the char 6 so that string afterwards is not seen in plot
   name = strsplit(name_format,'6');
   
   %only use the name
   bar_names(n) = {name{1}(1:end-1)}; 
   bar_comp_data = [ bar_comp_data; (comparison.steps.nr_steps-60)./60*100];
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
title("step counting error of 60 steps") 
ylabel('error (%)') 

%% Absolute number of sd.steps detected

bar_comp_data = [];
bar_names = {};
n = 0;
figure(1)
for comparison = sd_og_data_comparisons'
   n = n + 1;
   % formatting name for nice plotting by replacing 
   name_format = strrep(comparison.name,'_',' ');
   
   %cut the string at the char 5 so that date info is not seen in plot
   name = strsplit(name_format,'H');
   
   %only use the name
   bar_names(n) = {name{1}(1:end-1)}; 
   bar_comp_data = [ bar_comp_data; comparison.steps.nr_steps];    
end
bar_names = categorical(bar_names);
bar(bar_names,bar_comp_data)
title('step counting comparison with original data') 

%% All step detection component plots

close all

for i = 1:length(sd_og_data_comparisons)
    figure(i)
    stackedplot(sd_og_data_comparisons(i).sd_components);
    title(sd_og_data_comparisons(i).name);
end

spreadfigures

%% Individual step detection component plot
close all
file.directory = path.path ;
file.name = 'backpocket1_60_steps_HIMU-2020-04-20_10-02-03.csv';

target.file = file;
target.dataSetProp = DataSetProp("Accelerometer","Time",(path.time_unit), ...
    ["X","Y","Z"]);

[target.steps, target.Acceleration, target.sd_components] = stepDetection(target,'file', false);

figure()
stackedplot(target.sd_components)

clear figure(2)
figure(2)
hax=axes;

hold on
plot(target.sd_components.Time,target.sd_components.acc1_conv_gauss)
scatter(target.steps.data.Time, target.steps.data.acc4_builtin_max)
hold off

%%

delta_t = seconds(diff(sd.Acceleration.Time));

histogram(delta_t)