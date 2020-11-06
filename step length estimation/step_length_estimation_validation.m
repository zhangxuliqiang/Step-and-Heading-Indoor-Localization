clc 

pocket_data_180m_index = contains({pocket_data.folder},'test_path2');

pocket_data_180m = pocket_data(pocket_data_180m_index);

sl_val_comparisons = [];

for sl_dataset = pocket_data_180m'
    
    file.directory = [sl_dataset.folder '/'] ;
    file.name = sl_dataset.name;
    
    disp(['step length estimation with :' file.name])
    
    target.file = file;
    target.dataSetProp = DataSetProp("Accelerometer","Time",(1E-2), ...
        ["X","Y","Z","or_X","or_Y","or_Z"]);
    
    [sl_val.properties, sl_val.raw_data] = JSONFile2Timetable(target);
    
    find_person_index = strfind(sl_dataset.folder,'person');
    str_split = strsplit(sl_dataset.folder(find_person_index:end),'/');
    sl_val.properties.user_id = convertCharsToStrings(str_split{1});
    
    disp(sl_val.properties.user_id)
    
    % vezocnik specific data massage since 
    sl_val.raw_data.acc0_magnitude = nan(height(sl_val.raw_data),1);
    sl_val.raw_data.acc0_magnitude = sqrt(sl_val.raw_data.X.^2 + (-sl_val.raw_data.Z-9.81).^2 + ...
                                      sl_val.raw_data.Y.^2);
    
    [sl_val.steps, ~, sl_val.sd_components] = ...
        stepDetection( sl_val.raw_data,'data', false);
    
    sl_val.sl_components.period = [0; seconds(diff(sl_val.steps.data.Time))];
    sl_val.sl_components.period(sl_val.sl_components.period == 0) = nan;
    sl_val.sl_components.freq = 1./sl_val.sl_components.period;
    sl_val.mean_freq = mean(sl_val.sl_components.freq,'omitnan');
    
%     if strcmp(sl_val.properties.gender,"female")
%        sl_parameter = 0.3462;
%     else
%        sl_parameter = 0.2982; 
%     end
    
    sl_parameter = tian_constant;
    
    sl_val.step_length = sl_val.properties.height.*sl_parameter*sqrt(sl_val.sl_components.freq);
     sl_val.step_length(1) = sl_val.properties.height.*0.4;
    sl_val.distance_travelled = sum(sl_val.step_length,'omitnan');
    sl_val.distance_error = sl_val.distance_travelled - sl_val.properties.path_length;
    sl_val.distance_error_perc = 100*(sl_val.distance_error/sl_val.properties.path_length);
    
    sl_val_comparisons = [sl_val_comparisons; sl_val];
    
end

%%
clc
all_dist_error_perc = [sl_val_comparisons.distance_error_perc];
all_dist_error_perc([30,42]) = nan;

mean(abs(all_dist_error_perc),'omitnan')
std(abs(all_dist_error_perc),'omitnan')
%%

figure
subplot(2,1,1)
hold on
plot(abs(all_dist_error_perc_all_data))
plot(abs(all_dist_error_perc))
scatter(42,all_dist_error_perc(42),'xg')
hold off
title('Step Length Estimation Validation')
xlabel('Dataset ID (#)')
ylabel('absolute percentage distance error (%)')
leg1 = legend('K = 0.3116','K = 0.3080','Location','NorthWest');
title(leg1,'Tunable Parameter')
subplot(2,1,2)
plot(abs(all_dist_error_perc_all_data) - abs(all_dist_error_perc))



